#!/usr/bin/env python3
"""
D* Lite Global Planner Node for Nav2 (ROS2)

Implements the ComputePathToPose action server interface expected by
bt_navigator, replacing the NavfnPlanner/planner_server combination.

Subscribes:
  /global_costmap/costmap  [nav_msgs/OccupancyGrid]

TF:
  Looks up base_footprint in map frame to get robot position

Action Server:
  compute_path_to_pose     [nav2_msgs/action/ComputePathToPose]

Publishes:
  /dstar_path              [nav_msgs/Path]  (RViz visualization)
"""

import math
import time
import threading

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose

import tf2_ros

from fyp_dstar_lite.grid import OccupancyGridMap
from fyp_dstar_lite.d_star_lite import DStarLite


class DStarPlannerNode(Node):
    def __init__(self):
        super().__init__('dstar_planner_node')

        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('lethal_threshold', 65)
        self.declare_parameter('use_costmap', True)

        self._costmap_topic = self.get_parameter('costmap_topic').value
        self._lethal_threshold = self.get_parameter('lethal_threshold').value

        self._current_costmap: OccupancyGrid | None = None
        self._costmap_lock = threading.Lock()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cb_group = ReentrantCallbackGroup()

        # Nav2 global costmap publishes with TRANSIENT_LOCAL / RELIABLE / KEEP_LAST(5)
        # Match exactly so ROS2 QoS negotiation succeeds.
        transient_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._costmap_sub = self.create_subscription(
            OccupancyGrid,
            self._costmap_topic,
            self._costmap_callback,
            qos_profile=transient_qos,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'Subscribed to {self._costmap_topic} with TRANSIENT_LOCAL QoS'
        )

        self._path_pub = self.create_publisher(Path, '/dstar_path', 10)

        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self._plan_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'D* Lite Planner Node started — listening on {self._costmap_topic}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _costmap_callback(self, msg: OccupancyGrid):
        with self._costmap_lock:
            self._current_costmap = msg
        self.get_logger().info(
            f'Costmap received: {msg.info.width}x{msg.info.height} cells, '
            f'res={msg.info.resolution:.4f} m, '
            f'origin=({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})',
            once=True,
        )

    def _goal_callback(self, _goal_request):
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _plan_callback(self, goal_handle):
        request = goal_handle.request
        result = ComputePathToPose.Result()

        goal_x = request.goal.pose.position.x
        goal_y = request.goal.pose.position.y
        self.get_logger().info(f'Planning to ({goal_x:.2f}, {goal_y:.2f})')

        # Wait up to 10 s for costmap (executor handles callbacks on other threads)
        if self._current_costmap is None:
            self.get_logger().warn('Waiting for costmap (up to 10 s)...')
            deadline = time.monotonic() + 10.0
            while self._current_costmap is None:
                if time.monotonic() > deadline:
                    self.get_logger().error('Costmap never arrived — aborting goal.')
                    goal_handle.abort()
                    return result
                time.sleep(0.1)

        with self._costmap_lock:
            costmap_msg = self._current_costmap

        # ---- Determine start pose ----------------------------------------
        if request.use_start:
            start_world = (
                request.start.pose.position.x,
                request.start.pose.position.y,
            )
        else:
            try:
                tf = self._tf_buffer.lookup_transform(
                    costmap_msg.header.frame_id,
                    'base_footprint',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0),
                )
                start_world = (
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                )
            except Exception as exc:
                self.get_logger().error(f'TF lookup failed: {exc}')
                goal_handle.abort()
                return result

        goal_world = (goal_x, goal_y)

        # ---- World ↔ grid helpers ----------------------------------------
        info = costmap_msg.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        res = info.resolution
        width = info.width
        height = info.height

        def w2g(wx, wy):
            gx = int((wx - ox) / res)
            gy = int((wy - oy) / res)
            gx = max(0, min(width - 1, gx))
            gy = max(0, min(height - 1, gy))
            return (gx, gy)

        def g2w(gx, gy):
            return (ox + (gx + 0.5) * res, oy + (gy + 0.5) * res)

        start_grid = w2g(*start_world)
        goal_grid = w2g(*goal_world)

        self.get_logger().info(
            f'Grid: start={start_grid}, goal={goal_grid}, map={width}x{height}'
        )

        # ---- Build OccupancyGridMap from Nav2 costmap --------------------
        # Nav2 costmap values: 0=free, 1-252=inflation, 253=inscribed,
        #                      254=lethal, 255=unknown, -1=unknown (int8)
        ogrid = OccupancyGridMap(x_dim=width, y_dim=height, exploration_setting='8N')
        raw = np.array(costmap_msg.data, dtype=np.int16).reshape((height, width))

        # costmap is stored row-major (y, x); OccupancyGridMap indexes as [x][y]
        # Mark cells that are lethal or unknown as obstacles
        lethal_mask = (raw >= self._lethal_threshold) | (raw < 0)
        xs, ys = np.where(lethal_mask.T)  # transpose to get (x, y) pairs
        for gx, gy in zip(xs.tolist(), ys.tolist()):
            ogrid.set_obstacle((gx, gy))

        # Ensure start and goal cells are not marked as obstacles
        # (they might be inside inflation — clear them so a path can start/end)
        ogrid.remove_obstacle(start_grid)
        ogrid.remove_obstacle(goal_grid)

        # ---- Run D* Lite -------------------------------------------------
        try:
            dstar = DStarLite(map=ogrid, s_start=start_grid, s_goal=goal_grid)
            dstar.sensed_map = ogrid  # full map known upfront; no incremental sensing
            dstar.new_edges_and_old_costs = None

            path_grid, _, _ = dstar.move_and_replan(robot_position=start_grid)

        except AssertionError as exc:
            self.get_logger().error(f'D* Lite: no path found — {exc}')
            goal_handle.abort()
            return result
        except ValueError as exc:
            self.get_logger().error(f'D* Lite: {exc}')
            goal_handle.abort()
            return result
        except Exception as exc:
            self.get_logger().error(f'D* Lite: unexpected error — {exc}')
            goal_handle.abort()
            return result

        # ---- Post-process: prune redundant waypoints ---------------------
        # 1. Line-of-sight shortcut: skip waypoints that are directly visible
        path_grid = _los_prune(path_grid, ogrid)
        # 2. RDP simplification: collapse near-collinear points into straight segments
        # epsilon=3.0 grid cells (~15cm) — fewer waypoints = smoother DWB tracking
        path_grid = _rdp_simplify(path_grid, epsilon=3.0)

        # ---- Convert grid path → nav_msgs/Path ---------------------------
        now = self.get_clock().now().to_msg()
        frame = costmap_msg.header.frame_id

        path_msg = Path()
        path_msg.header.frame_id = frame
        path_msg.header.stamp = now

        for (gx, gy) in path_grid:
            wx, wy = g2w(gx, gy)
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp = now
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        # Fill yaw: each pose points toward the next waypoint
        _fill_path_orientations(path_msg, request.goal.pose.orientation)

        self._path_pub.publish(path_msg)

        result.path = path_msg
        result.planning_time = Duration(sec=0, nanosec=0)
        goal_handle.succeed()

        self.get_logger().info(
            f'D* Lite path found: {len(path_grid)} waypoints'
        )
        return result


def _los_clear(p1, p2, grid) -> bool:
    """Bresenham line-of-sight check between two grid cells. Returns True if clear."""
    x0, y0 = p1
    x1, y1 = p2
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        if not grid.is_unoccupied((x, y)):
            return False
        if x == x1 and y == y1:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def _los_prune(path, grid) -> list:
    """Greedy line-of-sight waypoint pruning: skip intermediate points that
    are directly visible from the current anchor."""
    if len(path) <= 2:
        return path
    pruned = [path[0]]
    i = 0
    while i < len(path) - 1:
        # Find the furthest point reachable in a straight line from path[i]
        j = len(path) - 1
        while j > i + 1:
            if _los_clear(path[i], path[j], grid):
                break
            j -= 1
        pruned.append(path[j])
        i = j
    return pruned


def _rdp_simplify(path, epsilon: float = 1.5) -> list:
    """Ramer-Douglas-Peucker path simplification.
    Removes points that deviate less than epsilon cells from a straight line."""
    if len(path) <= 2:
        return path

    def point_line_dist(p, a, b):
        ax, ay = a
        bx, by = b
        px, py = p
        dx, dy = bx - ax, by - ay
        if dx == 0 and dy == 0:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

    def rdp(pts):
        if len(pts) <= 2:
            return pts
        max_dist = 0.0
        idx = 0
        for i in range(1, len(pts) - 1):
            d = point_line_dist(pts[i], pts[0], pts[-1])
            if d > max_dist:
                max_dist = d
                idx = i
        if max_dist > epsilon:
            left = rdp(pts[:idx + 1])
            right = rdp(pts[idx:])
            return left[:-1] + right
        return [pts[0], pts[-1]]

    return rdp(path)


def _fill_path_orientations(path: Path, goal_orientation=None):
    poses = path.poses
    for i in range(len(poses) - 1):
        dx = poses[i + 1].pose.position.x - poses[i].pose.position.x
        dy = poses[i + 1].pose.position.y - poses[i].pose.position.y
        yaw = math.atan2(dy, dx)
        poses[i].pose.orientation.x = 0.0
        poses[i].pose.orientation.y = 0.0
        poses[i].pose.orientation.z = math.sin(yaw / 2.0)
        poses[i].pose.orientation.w = math.cos(yaw / 2.0)
    if poses:
        if goal_orientation is not None:
            poses[-1].pose.orientation = goal_orientation
        elif len(poses) > 1:
            poses[-1].pose.orientation = poses[-2].pose.orientation


def main(args=None):
    rclpy.init(args=args)
    node = DStarPlannerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
