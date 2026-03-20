import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import csv

class PathFollower(Node):
    def __init__(self, path, resolution):
        super().__init__('path_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path = path
        self.resolution = resolution
        self.index = 0
        self.current_pose = None
        self.timer = self.create_timer(0.1, self.follow_path)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def follow_path(self):
        if self.current_pose is None or self.index >= len(self.path):
            return

        target = self.path[self.index]
        tx = target[1] * self.resolution
        ty = target[0] * self.resolution

        dx = tx - self.current_pose.position.x
        dy = ty - self.current_pose.position.y
        dist = math.sqrt(dx**2 + dy**2)

        angle = math.atan2(dy, dx)

        if dist < 0.1:
            self.index += 1
            return

        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = angle  # Basic turning control
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    path = []
    with open("planned_path.csv", "r") as f:
        reader = csv.reader(f)
        for row in reader:
            path.append((int(row[0]), int(row[1])))

    resolution = 0.1  # meters per grid cell
    node = PathFollower(path, resolution)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
