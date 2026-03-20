#!/usr/bin/env python3
"""
Obstacle Gate Node — stops the robot when an obstacle is detected in front.

Sits between velocity_smoother and the robot:
  /cmd_vel_smoothed → [obstacle_gate] → /cmd_vel

Subscribes to /scan and checks if any laser reading within a front cone
is closer than stop_distance. If so, publishes zero velocity.
When the obstacle clears, forwards velocity commands normally.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleGateNode(Node):
    def __init__(self):
        super().__init__('obstacle_gate')

        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('cone_angle', 0.524)  # ±30 degrees

        self._stop_dist = self.get_parameter('stop_distance').value
        self._cone_angle = self.get_parameter('cone_angle').value
        self._blocked = False
        self._latest_scan: LaserScan | None = None

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)

        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_subscription(Twist, 'cmd_vel_in', self._vel_cb, 10)

        self.get_logger().info(
            f'Obstacle gate: stop_distance={self._stop_dist}m, '
            f'cone=±{math.degrees(self._cone_angle):.0f}°'
        )

    def _scan_cb(self, msg: LaserScan):
        self._latest_scan = msg

    def _vel_cb(self, msg: Twist):
        blocked = self._check_blocked()

        if blocked and not self._blocked:
            self.get_logger().warn('Obstacle detected — stopping')
        elif not blocked and self._blocked:
            self.get_logger().info('Path clear — resuming')
        self._blocked = blocked

        if blocked:
            self._cmd_pub.publish(Twist())  # zero velocity
        else:
            self._cmd_pub.publish(msg)

    def _check_blocked(self) -> bool:
        scan = self._latest_scan
        if scan is None:
            return False

        angle = scan.angle_min
        for r in scan.ranges:
            if abs(angle) <= self._cone_angle:
                if scan.range_min < r < self._stop_dist:
                    return True
            angle += scan.angle_increment

        return False


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
