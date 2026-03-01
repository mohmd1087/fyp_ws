#!/usr/bin/env python3
"""
Syncs RViz 2D Pose Estimate to Gazebo robot position.
Subscribes to /initialpose and teleports the robot in Gazebo.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import subprocess
import math


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PoseSyncNode(Node):
    def __init__(self):
        super().__init__('pose_sync')

        self.declare_parameter('robot_name', 'robot')
        self.declare_parameter('world_name', 'default')

        self.robot_name = self.get_parameter('robot_name').value
        self.world_name = self.get_parameter('world_name').value

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )

        self.get_logger().info(f'Pose sync node started. Robot: {self.robot_name}, World: {self.world_name}')

    def initialpose_callback(self, msg):
        """Handle incoming initial pose from RViz."""
        pose = msg.pose.pose

        x = pose.position.x
        y = pose.position.y
        z = 0.1  # Slightly above ground to avoid collision issues

        # Convert quaternion to euler for yaw
        roll, pitch, yaw = quaternion_to_euler(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        self.get_logger().info(f'Teleporting robot to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        # Use gz service to set pose
        # Format: gz service -s /world/<world>/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req 'name: "robot", position: {x: 1, y: 2, z: 0.1}, orientation: {x: 0, y: 0, z: 0, w: 1}'

        cmd = [
            'gz', 'service', '-s', f'/world/{self.world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'name: "{self.robot_name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: {pose.orientation.x}, y: {pose.orientation.y}, z: {pose.orientation.z}, w: {pose.orientation.w}}}'
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self.get_logger().info('Robot teleported successfully')
            else:
                self.get_logger().error(f'Failed to teleport: {result.stderr}')
        except subprocess.TimeoutExpired:
            self.get_logger().error('Teleport command timed out')
        except Exception as e:
            self.get_logger().error(f'Error teleporting robot: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseSyncNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
