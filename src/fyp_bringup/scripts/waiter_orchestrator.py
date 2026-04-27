#!/usr/bin/env python3
"""
Waiter Robot Orchestrator Node

State machine that dispatches the robot to tables, manages LiveKit voice agent
rooms, and returns the robot home after order confirmation.

HTTP API (port 5050):
  POST /dispatch        {"table_id": "table-1"}
  POST /order_complete  {"room_name": "table-1", "order_id": "..."}
  GET  /status          -> {"state": "IDLE", "current_table": null}

ROS2:
  Publishes /waiter/state  (std_msgs/String)
  Uses Nav2 BasicNavigator for goal sending
"""

import json
import math
import os
import threading
import time
import asyncio
import urllib.request
from enum import Enum
from functools import partial
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path

from dotenv import load_dotenv

import yaml
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# LiveKit Server SDK is optional — room creation is skipped if not installed
try:
    from livekit import api as livekit_api
    HAS_LIVEKIT = True
except ImportError:
    HAS_LIVEKIT = False

# Pysher is optional — remote dispatch via Pusher is skipped if not installed
try:
    import pysher
    HAS_PYSHER = True
except ImportError:
    HAS_PYSHER = False


# Safety net: auto-release AT_TABLE after this many seconds so a missed voice
# agent callback, stuck tray ESP, or crashed dashboard can't strand the robot.
AT_TABLE_TIMEOUT_SEC = 300.0


class WaiterState(Enum):
    IDLE = "IDLE"
    NAVIGATING_TO_TABLE = "NAVIGATING_TO_TABLE"
    AT_TABLE = "AT_TABLE"
    AT_TABLE_FOLLOWUP = "AT_TABLE_FOLLOWUP"
    NAVIGATING_HOME = "NAVIGATING_HOME"


def yaw_to_quaternion(yaw: float):
    """Convert yaw (radians) to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose(x: float, y: float, yaw: float, navigator: BasicNavigator) -> PoseStamped:
    """Create a PoseStamped in the map frame."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class WaiterOrchestrator:
    """
    Orchestrator that wraps BasicNavigator (which is already a ROS2 Node).
    No Node subclassing — avoids dual-node conflicts.
    """

    def __init__(self, navigator: BasicNavigator, config: dict):
        self.navigator = navigator
        self.logger = navigator.get_logger()

        # Table positions
        self.home_pose_raw = config["home"]
        self.table_poses_raw = config["tables"]
        self.logger.info(
            f"Loaded {len(self.table_poses_raw)} tables: {list(self.table_poses_raw.keys())}"
        )

        # LiveKit credentials
        self.lk_url = os.environ.get("LIVEKIT_URL", "")
        self.lk_api_key = os.environ.get("LIVEKIT_API_KEY", "")
        self.lk_api_secret = os.environ.get("LIVEKIT_API_SECRET", "")

        # State machine (all access guarded by _lock)
        self._lock = threading.Lock()
        self._state = WaiterState.IDLE
        self._current_table: str | None = None
        self._current_room: str | None = None
        self._nav2_ready = False

        # Signals from HTTP thread
        self._dispatch_event = threading.Event()
        self._dispatch_table_id: str | None = None
        self._dispatch_tray: int | None = None
        self._dispatch_order_id: str | None = None
        self._current_tray: int | None = None
        self._current_order_id: str | None = None
        self._order_complete_event = threading.Event()
        self._go_home_event = threading.Event()
        self._at_table_entered_at: float | None = None

        # State publisher
        self._state_pub = navigator.create_publisher(String, "/waiter/state", 10)

        # Tray command publisher (signals ESP32 to open a tray on arrival)
        self._tray_pub = navigator.create_publisher(Int32, "/waiter/tray_cmd", 10)

        # Tray-closed feedback from the micro-ROS ESP32. Motor ESP asserts a
        # GPIO line when the tray physically closes; firmware republishes the
        # last-commanded tray number here. Treated as an alternate path to
        # release AT_TABLE (HTTP /order_complete remains a fallback).
        self._tray_status_sub = navigator.create_subscription(
            Int32, "/waiter/tray_status", self._on_tray_status, 10
        )

        # Timer to drive state machine (1 Hz). Use a ReentrantCallbackGroup so
        # nested spin calls inside BasicNavigator.goToPose() don't deadlock the
        # executor.
        self._cb_group = ReentrantCallbackGroup()
        navigator.create_timer(1.0, self._tick, callback_group=self._cb_group)

        # HTTP server (daemon thread)
        self._http_server = HTTPServer(
            ("0.0.0.0", 5050), partial(_HTTPHandler, orchestrator=self)
        )
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever, daemon=True
        )
        self._http_thread.start()
        self.logger.info("HTTP server listening on port 5050")

        # Dashboard status publishing (via Vercel API route → Pusher)
        self._dashboard_url = os.environ.get("DASHBOARD_URL", "")
        self._agent_api_key = os.environ.get("AGENT_API_KEY", "")
        self._last_published_state: str | None = None

        # Pusher subscription for remote dispatch commands from dashboard
        if HAS_PYSHER:
            pusher_key = os.environ.get("PUSHER_KEY", "65f267cd383ad90fde17")
            pusher_cluster = os.environ.get("PUSHER_CLUSTER", "ap2")
            self._pusher_client = pysher.Pusher(pusher_key, cluster=pusher_cluster)
            self._pusher_client.connection.bind(
                "pusher:connection_established", self._on_pusher_connected
            )
            self._pusher_client.connect()
            self.logger.info("Connecting to Pusher for remote dispatch...")
        else:
            self._pusher_client = None
            self.logger.warn(
                "pysher not installed — remote dispatch from dashboard disabled. "
                "Install with: pip install pysher"
            )

    # ------------------------------------------------------------------
    # Pusher integration (remote dispatch + status publishing)
    # ------------------------------------------------------------------
    def _on_pusher_connected(self, data):
        """Called when Pusher WebSocket connects/reconnects."""
        channel = self._pusher_client.subscribe("robot")
        channel.bind("robot.dispatch", self._on_remote_dispatch)
        channel.bind("robot.go_home", self._on_remote_go_home)
        self.logger.info("Subscribed to Pusher 'robot' channel for remote dispatch")

    def _on_remote_dispatch(self, data):
        """Handle dispatch command from dashboard via Pusher."""
        try:
            payload = json.loads(data) if isinstance(data, str) else data
            table_id = payload.get("table_id")
            tray = payload.get("tray")  # None if not specified (original buttons)
            order_id = payload.get("order_id")  # None for non-delivery dispatches
            if table_id:
                result = self.handle_dispatch(table_id, tray=tray, order_id=order_id)
                self.logger.info(f"Remote dispatch result: {result}")
        except Exception as e:
            self.logger.error(f"Failed to handle remote dispatch: {e}")

    def _on_remote_go_home(self, data):
        """Handle go-home command from dashboard via Pusher."""
        try:
            result = self.handle_go_home()
            self.logger.info(f"Remote go_home result: {result}")
        except Exception as e:
            self.logger.error(f"Failed to handle remote go_home: {e}")

    def _publish_status_to_dashboard(self, state: str, current_table: str | None):
        """POST status to dashboard API route, which republishes via Pusher."""
        try:
            url = f"{self._dashboard_url}/api/robot/status"
            payload = json.dumps({
                "state": state,
                "current_table": current_table,
            }).encode()
            req = urllib.request.Request(
                url,
                data=payload,
                headers={
                    "Content-Type": "application/json",
                    "x-agent-key": self._agent_api_key,
                },
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=5):
                pass
        except Exception as e:
            self.logger.warn(f"Failed to publish status to dashboard: {e}")

    def mark_nav2_ready(self):
        with self._lock:
            self._nav2_ready = True
        self.logger.info("Nav2 is active. Orchestrator accepting commands.")

    def _send_goal_nonblocking(self, pose: PoseStamped):
        """Send a NavigateToPose goal WITHOUT calling spin_until_future_complete.

        BasicNavigator.goToPose() blocks the executor via a nested spin waiting
        for goal acceptance, which deadlocks when called from a timer callback.
        We bypass that by calling the action client directly and tracking the
        futures ourselves in _check_nav_result().
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ""

        # Kick off send_goal_async; do NOT wait for it. The result-future chain
        # is wired up in a callback so result_future gets set as soon as the
        # server accepts.
        self.navigator.goal_handle = None
        self.navigator.result_future = None
        send_goal_future = self.navigator.nav_to_pose_client.send_goal_async(
            goal_msg, feedback_callback=self.navigator._feedbackCallback
        )
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        """Callback when the action server accepts/rejects the goal."""
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.logger.error("Goal was rejected by server")
            with self._lock:
                self._state = WaiterState.IDLE
                self._current_table = None
            return
        self.logger.info("Goal accepted — awaiting result")
        self.navigator.goal_handle = goal_handle
        self.navigator.result_future = goal_handle.get_result_async()

    def _check_nav_result(self) -> TaskResult | None:
        """Check navigation result WITHOUT nested rclpy.spin (avoids deadlock).

        BasicNavigator.isTaskComplete() calls spin_until_future_complete()
        internally, which deadlocks when called from a timer callback running
        inside rclpy.spin().  Instead we check the future directly.

        Returns TaskResult if done, None if still in progress.
        """
        result_future = self.navigator.result_future
        if result_future is None:
            return TaskResult.UNKNOWN

        if not result_future.done():
            return None  # still navigating

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    # ------------------------------------------------------------------
    # State machine tick (runs on ROS2 timer thread)
    # ------------------------------------------------------------------
    def _tick(self):
        with self._lock:
            state = self._state
            current_table = self._current_table

        # Publish current state (ROS2 topic)
        msg = String()
        msg.data = json.dumps({"state": state.value, "current_table": current_table})
        self._state_pub.publish(msg)

        # Publish state changes to dashboard via Pusher (non-blocking)
        state_key = f"{state.value}:{current_table}"
        if state_key != self._last_published_state and self._dashboard_url:
            self._last_published_state = state_key
            threading.Thread(
                target=self._publish_status_to_dashboard,
                args=(state.value, current_table),
                daemon=True,
            ).start()

        # --- IDLE: check for dispatch or go-home request ---
        if state == WaiterState.IDLE:
            if self._dispatch_event.is_set():
                self._dispatch_event.clear()
                table_id = self._dispatch_table_id
                if table_id and table_id in self.table_poses_raw:
                    t = self.table_poses_raw[table_id]
                    goal = make_pose(t["x"], t["y"], t["yaw"], self.navigator)
                    self.logger.info(f"Navigating to {table_id} ({t['x']}, {t['y']})")
                    self._send_goal_nonblocking(goal)
                    with self._lock:
                        self._current_table = table_id
                        self._state = WaiterState.NAVIGATING_TO_TABLE
                else:
                    self.logger.warn(f"Unknown table_id: {table_id}")
            elif self._go_home_event.is_set():
                self._go_home_event.clear()
                h = self.home_pose_raw
                goal = make_pose(h["x"], h["y"], h["yaw"], self.navigator)
                self.logger.info(f"Manual go-home: navigating to home ({h['x']}, {h['y']})")
                self._send_goal_nonblocking(goal)
                with self._lock:
                    self._state = WaiterState.NAVIGATING_HOME

        # --- NAVIGATING_TO_TABLE: poll nav completion ---
        elif state == WaiterState.NAVIGATING_TO_TABLE:
            result = self._check_nav_result()
            if result is not None:
                self.logger.info(f"Nav task complete. result={result}")
                if result == TaskResult.SUCCEEDED:
                    self.logger.info(f"Arrived at {current_table}!")
                    with self._lock:
                        self._state = WaiterState.AT_TABLE
                        self._current_tray = self._dispatch_tray
                        self._current_order_id = self._dispatch_order_id
                        self._at_table_entered_at = time.monotonic()
                    self._on_arrived_at_table()
                else:
                    self.logger.error(f"Navigation to {current_table} failed: {result}")
                    with self._lock:
                        self._state = WaiterState.IDLE
                        self._current_table = None

        # --- AT_TABLE: wait for order_complete signal ---
        elif state == WaiterState.AT_TABLE:
            entered_at = self._at_table_entered_at
            if (
                entered_at is not None
                and not self._order_complete_event.is_set()
                and (time.monotonic() - entered_at) > AT_TABLE_TIMEOUT_SEC
            ):
                self.logger.warn(
                    f"AT_TABLE timeout ({AT_TABLE_TIMEOUT_SEC:.0f}s) "
                    f"— auto-releasing to return home"
                )
                self._order_complete_event.set()

            if self._order_complete_event.is_set():
                self._order_complete_event.clear()
                self.logger.info("Order confirmed! Returning home.")
                h = self.home_pose_raw
                goal = make_pose(h["x"], h["y"], h["yaw"], self.navigator)
                self._send_goal_nonblocking(goal)
                with self._lock:
                    self._state = WaiterState.NAVIGATING_HOME
                    self._at_table_entered_at = None

        # --- AT_TABLE_FOLLOWUP: wait for order_complete OR decline_followup ---
        elif state == WaiterState.AT_TABLE_FOLLOWUP:
            entered_at = self._at_table_entered_at
            if (
                entered_at is not None
                and not self._order_complete_event.is_set()
                and (time.monotonic() - entered_at) > AT_TABLE_TIMEOUT_SEC
            ):
                self.logger.warn(
                    f"AT_TABLE_FOLLOWUP timeout ({AT_TABLE_TIMEOUT_SEC:.0f}s) "
                    f"— auto-releasing to return home"
                )
                self._order_complete_event.set()

            if self._order_complete_event.is_set():
                self._order_complete_event.clear()
                self.logger.info("Follow-up complete! Returning home.")
                h = self.home_pose_raw
                goal = make_pose(h["x"], h["y"], h["yaw"], self.navigator)
                self._send_goal_nonblocking(goal)
                with self._lock:
                    self._state = WaiterState.NAVIGATING_HOME
                    self._at_table_entered_at = None

        # --- NAVIGATING_HOME: poll nav completion ---
        elif state == WaiterState.NAVIGATING_HOME:
            result = self._check_nav_result()
            if result is not None:
                if result == TaskResult.SUCCEEDED:
                    self.logger.info("Returned home. Ready for next dispatch.")
                else:
                    self.logger.warn(f"Return home result: {result}")
                with self._lock:
                    self._current_table = None
                    self._current_room = None
                    self._current_tray = None
                    self._current_order_id = None
                    self._at_table_entered_at = None
                    self._state = WaiterState.IDLE

    # ------------------------------------------------------------------
    # LiveKit room management
    # ------------------------------------------------------------------
    def _on_arrived_at_table(self):
        """Create/ensure LiveKit room exists for this table, and signal tray if requested."""
        with self._lock:
            room_name = self._current_table  # e.g. "table-1"
            self._current_room = room_name
            tray = self._current_tray

        # Publish tray command to ESP32 (only if a tray was selected)
        if tray is not None:
            tray_msg = Int32()
            tray_msg.data = tray
            self._tray_pub.publish(tray_msg)
            self.logger.info(f"Published tray command: tray {tray}")

        if not HAS_LIVEKIT:
            self.logger.warn("livekit-api not installed, skipping room creation")
            return

        if not self.lk_url or not self.lk_api_key:
            self.logger.warn("LiveKit credentials not set, skipping room creation")
            return

        # Run async room creation in a background thread
        threading.Thread(
            target=self._create_room_sync, args=(room_name,), daemon=True
        ).start()

    def _on_tray_status(self, msg: Int32):
        """Handle tray-closed feedback from the ESP32.

        Delivery (tray dispatched): AT_TABLE → AT_TABLE_FOLLOWUP so the voice
        agent can ask 'anything else?'.
        Non-delivery (defensive): release AT_TABLE to return home.
        """
        with self._lock:
            state = self._state
            expected = self._current_tray
            room_name = self._current_room
        if state != WaiterState.AT_TABLE:
            return
        if expected is not None and msg.data != expected:
            self.logger.warn(
                f"Ignoring tray_status={msg.data}; expected tray {expected}"
            )
            return

        if expected is not None:
            self.logger.info(
                f"Tray {msg.data} closed; transitioning AT_TABLE → AT_TABLE_FOLLOWUP."
            )
            with self._lock:
                self._state = WaiterState.AT_TABLE_FOLLOWUP
                self._at_table_entered_at = time.monotonic()
            if room_name and HAS_LIVEKIT and self.lk_url and self.lk_api_key:
                threading.Thread(
                    target=self._create_room_sync, args=(room_name,), daemon=True
                ).start()
        else:
            self.logger.info(
                f"Tray {msg.data} closed (no tray dispatched); releasing AT_TABLE."
            )
            self._order_complete_event.set()

    def _create_room_sync(self, room_name: str):
        """Create the LiveKit room (runs in a thread with its own event loop)."""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._create_room_async(room_name))
            loop.close()
        except Exception as e:
            self.logger.error(f"Failed to create LiveKit room: {e}")

    async def _create_room_async(self, room_name: str):
        """Create the LiveKit room via Server SDK."""
        lk = livekit_api.LiveKitAPI(
            self.lk_url,
            self.lk_api_key,
            self.lk_api_secret,
        )
        try:
            await lk.room.create_room(
                livekit_api.CreateRoomRequest(
                    name=room_name,
                    empty_timeout=3600,
                )
            )
            self.logger.info(f"LiveKit room '{room_name}' ready")
        finally:
            await lk.aclose()

    # ------------------------------------------------------------------
    # HTTP dispatch / order_complete handlers (called from HTTP thread)
    # ------------------------------------------------------------------
    def handle_dispatch(
        self,
        table_id: str,
        tray: int | None = None,
        order_id: str | None = None,
    ) -> dict:
        with self._lock:
            if not self._nav2_ready:
                return {"ok": False, "error": "Nav2 is not ready yet"}
            if self._state != WaiterState.IDLE:
                return {
                    "ok": False,
                    "error": f"Robot is busy ({self._state.value})",
                    "current_table": self._current_table,
                }
            self._dispatch_table_id = table_id
            self._dispatch_tray = tray
            self._dispatch_order_id = order_id
            self._dispatch_event.set()
            return {"ok": True, "table_id": table_id, "tray": tray, "order_id": order_id}

    def handle_go_home(self) -> dict:
        with self._lock:
            if not self._nav2_ready:
                return {"ok": False, "error": "Nav2 is not ready yet"}
            if self._state != WaiterState.IDLE:
                return {
                    "ok": False,
                    "error": f"Robot is busy ({self._state.value})",
                    "current_table": self._current_table,
                }
            self._go_home_event.set()
            return {"ok": True}

    def handle_order_complete(self, room_name: str, order_id: str) -> dict:
        with self._lock:
            if self._state not in (WaiterState.AT_TABLE, WaiterState.AT_TABLE_FOLLOWUP):
                return {"ok": False, "error": f"Not at table ({self._state.value})"}
            self.logger.info(f"Order {order_id} complete in room {room_name}")
            self._order_complete_event.set()
            return {"ok": True}

    def handle_decline_followup(self, room_name: str) -> dict:
        with self._lock:
            if self._state != WaiterState.AT_TABLE_FOLLOWUP:
                return {"ok": False, "error": f"Not in follow-up ({self._state.value})"}
            self.logger.info(f"Customer declined follow-up in room {room_name}")
            self._order_complete_event.set()
            return {"ok": True}

    def get_status(self) -> dict:
        with self._lock:
            mode = "delivery" if self._current_tray is not None else "order"
            return {
                "state": self._state.value,
                "current_table": self._current_table,
                "current_room": self._current_room,
                "mode": mode,
                "order_id": self._current_order_id,
                "tables": list(self.table_poses_raw.keys()),
            }


# ======================================================================
# HTTP Request Handler
# ======================================================================
class _HTTPHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, orchestrator: WaiterOrchestrator, **kwargs):
        self.orchestrator = orchestrator
        super().__init__(*args, **kwargs)

    def _send_json(self, data: dict, status: int = 200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        """Handle CORS preflight requests."""
        self._send_json({}, 204)

    def do_GET(self):
        if self.path == "/status":
            self._send_json(self.orchestrator.get_status())
        else:
            self._send_json({"error": "Not found"}, 404)

    def do_POST(self):
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length) if content_length > 0 else b"{}"
        try:
            data = json.loads(body)
        except json.JSONDecodeError:
            self._send_json({"error": "Invalid JSON"}, 400)
            return

        if self.path == "/dispatch":
            table_id = data.get("table_id")
            if not table_id:
                self._send_json({"error": "table_id required"}, 400)
                return
            tray = data.get("tray")  # None if not specified (original buttons)
            order_id = data.get("order_id")  # None for non-delivery dispatches
            result = self.orchestrator.handle_dispatch(
                table_id, tray=tray, order_id=order_id
            )
            self._send_json(result, 200 if result["ok"] else 409)

        elif self.path == "/order_complete":
            room_name = data.get("room_name", "")
            order_id = data.get("order_id", "")
            result = self.orchestrator.handle_order_complete(room_name, order_id)
            self._send_json(result, 200 if result["ok"] else 409)

        elif self.path == "/decline_followup":
            room_name = data.get("room_name", "")
            result = self.orchestrator.handle_decline_followup(room_name)
            self._send_json(result, 200 if result["ok"] else 409)

        else:
            self._send_json({"error": "Not found"}, 404)

    def log_message(self, format, *args):
        """Suppress default HTTP logs — we use ROS2 logging."""
        pass


# ======================================================================
# Main
# ======================================================================
def main():
    # Load environment variables from orchestrator.env (same config/ directory)
    env_path = Path(__file__).resolve().parent.parent / "config" / "orchestrator.env"
    load_dotenv(env_path)

    rclpy.init()

    # BasicNavigator is the sole ROS2 Node in this process
    navigator = BasicNavigator("waiter_navigator")

    # Load config
    navigator.declare_parameter("config_file", "")
    config_path = navigator.get_parameter("config_file").get_parameter_value().string_value

    if not config_path or not os.path.exists(config_path):
        navigator.get_logger().fatal(f"Config file not found: {config_path}")
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # Create orchestrator (attaches timer + HTTP server to navigator)
    orchestrator = WaiterOrchestrator(navigator, config)

    # Wait for Nav2 to be fully active
    navigator.get_logger().info("Waiting for Nav2 to become active...")
    # Use 'bt_navigator' as localizer when running RTAB-Map (no AMCL lifecycle node)
    import os as _os
    _localizer = 'amcl' if _os.environ.get('LOCALIZATION_MODE', 'amcl').lower() == 'amcl' else 'bt_navigator'
    navigator.waitUntilNav2Active(localizer=_localizer)
    orchestrator.mark_nav2_ready()

    # Use a MultiThreadedExecutor so the state-machine timer, HTTP dispatches,
    # and the nested rclpy.spin_until_future_complete() calls inside
    # BasicNavigator.goToPose() can run concurrently without deadlocking.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(navigator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        navigator.lifecycleShutdown()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
