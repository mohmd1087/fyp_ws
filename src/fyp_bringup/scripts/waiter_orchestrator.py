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
import asyncio
import urllib.request
from enum import Enum
from functools import partial
from http.server import HTTPServer, BaseHTTPRequestHandler

import yaml
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
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


class WaiterState(Enum):
    IDLE = "IDLE"
    NAVIGATING_TO_TABLE = "NAVIGATING_TO_TABLE"
    AT_TABLE = "AT_TABLE"
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
        self._order_complete_event = threading.Event()

        # State publisher
        self._state_pub = navigator.create_publisher(String, "/waiter/state", 10)

        # Timer to drive state machine (1 Hz)
        navigator.create_timer(1.0, self._tick)

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
        self.logger.info("Subscribed to Pusher 'robot' channel for remote dispatch")

    def _on_remote_dispatch(self, data):
        """Handle dispatch command from dashboard via Pusher."""
        try:
            payload = json.loads(data) if isinstance(data, str) else data
            table_id = payload.get("table_id")
            if table_id:
                result = self.handle_dispatch(table_id)
                self.logger.info(f"Remote dispatch result: {result}")
        except Exception as e:
            self.logger.error(f"Failed to handle remote dispatch: {e}")

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

        # --- IDLE: check for dispatch request ---
        if state == WaiterState.IDLE:
            if self._dispatch_event.is_set():
                self._dispatch_event.clear()
                table_id = self._dispatch_table_id
                if table_id and table_id in self.table_poses_raw:
                    t = self.table_poses_raw[table_id]
                    goal = make_pose(t["x"], t["y"], t["yaw"], self.navigator)
                    self.logger.info(f"Navigating to {table_id} ({t['x']}, {t['y']})")
                    self.navigator.goToPose(goal)
                    with self._lock:
                        self._current_table = table_id
                        self._state = WaiterState.NAVIGATING_TO_TABLE
                else:
                    self.logger.warn(f"Unknown table_id: {table_id}")

        # --- NAVIGATING_TO_TABLE: poll nav completion ---
        elif state == WaiterState.NAVIGATING_TO_TABLE:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.logger.info(f"Arrived at {current_table}!")
                    with self._lock:
                        self._state = WaiterState.AT_TABLE
                    self._on_arrived_at_table()
                else:
                    self.logger.error(f"Navigation to {current_table} failed: {result}")
                    with self._lock:
                        self._state = WaiterState.IDLE
                        self._current_table = None

        # --- AT_TABLE: wait for order_complete signal ---
        elif state == WaiterState.AT_TABLE:
            if self._order_complete_event.is_set():
                self._order_complete_event.clear()
                self.logger.info("Order confirmed! Returning home.")
                h = self.home_pose_raw
                goal = make_pose(h["x"], h["y"], h["yaw"], self.navigator)
                self.navigator.goToPose(goal)
                with self._lock:
                    self._state = WaiterState.NAVIGATING_HOME

        # --- NAVIGATING_HOME: poll nav completion ---
        elif state == WaiterState.NAVIGATING_HOME:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.logger.info("Returned home. Ready for next dispatch.")
                else:
                    self.logger.warn(f"Return home result: {result}")
                with self._lock:
                    self._current_table = None
                    self._current_room = None
                    self._state = WaiterState.IDLE

    # ------------------------------------------------------------------
    # LiveKit room management
    # ------------------------------------------------------------------
    def _on_arrived_at_table(self):
        """Create/ensure LiveKit room exists for this table."""
        with self._lock:
            room_name = self._current_table  # e.g. "table-1"
            self._current_room = room_name

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
    def handle_dispatch(self, table_id: str) -> dict:
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
            self._dispatch_event.set()
            return {"ok": True, "table_id": table_id}

    def handle_order_complete(self, room_name: str, order_id: str) -> dict:
        with self._lock:
            if self._state != WaiterState.AT_TABLE:
                return {"ok": False, "error": f"Not at table ({self._state.value})"}
            self.logger.info(f"Order {order_id} complete in room {room_name}")
            self._order_complete_event.set()
            return {"ok": True}

    def get_status(self) -> dict:
        with self._lock:
            return {
                "state": self._state.value,
                "current_table": self._current_table,
                "current_room": self._current_room,
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
            result = self.orchestrator.handle_dispatch(table_id)
            self._send_json(result, 200 if result["ok"] else 409)

        elif self.path == "/order_complete":
            room_name = data.get("room_name", "")
            order_id = data.get("order_id", "")
            result = self.orchestrator.handle_order_complete(room_name, order_id)
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
    navigator.waitUntilNav2Active()
    orchestrator.mark_nav2_ready()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.lifecycleShutdown()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
