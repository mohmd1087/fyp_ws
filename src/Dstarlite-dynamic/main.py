import numpy as np
import time
import rclpy
import math
import threading
from gui import Animation
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from rclpy.task import Future
from rclpy.node import Node

OBSTACLE = 255
UNOCCUPIED = 0
FRONT_OFFSET = 0.0  # adjust if needed after testing

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CmdVelController(Node):
    def __init__(self, robot_name='bot_name10', resolution=0.1, grid_center=(50, 50)):
        super().__init__('cmd_vel_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(ModelStates, '/model_states', self.model_states_callback, 10)
        # 🔁 Manually fetch the latest /model_states once to initialize

        self.robot_name = robot_name
        self.robot_index = None
        self.position = None
        self.orientation = None
        self.resolution = resolution
        self.grid_center = grid_center
        self.path = []
        self.following = False 
        self.path_index = 0
        self.manual_fetch_model_states()
        
    def manual_fetch_model_states(self):
        future = Future()

        def once_callback(msg):
            if self.robot_name in msg.name:
                index = msg.name.index(self.robot_name)
                pose = msg.pose[index]
                self.position = (pose.position.x, pose.position.y)
                self.orientation = pose.orientation
                print(f"📦 Manually initialized from /model_states: {self.position}")
            else:
                print(f"⚠️ '{self.robot_name}' not in /model_states yet.")
            future.set_result(True)

        # Create a temporary one-shot subscription
        temp_sub = self.create_subscription(ModelStates, '/model_states', once_callback, 10)

        # Spin until we get the first result (max 2 seconds)
        timeout = time.time() + 2
        while not future.done() and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Destroy temp sub
        self.destroy_subscription(temp_sub)

     
    def model_states_callback(self, msg):
        try:
            if self.robot_name not in msg.name:
                return
            index = msg.name.index(self.robot_name)
            pose = msg.pose[index]
            self.position = (pose.position.x, pose.position.y)
            self.orientation = pose.orientation
            print(f"✅ Tracking {self.robot_name} → World Pos: {self.position}")
           # else:
           #     print(f"⚠️ Robot name '{self.robot_name}' not in /model_states list: {msg.name}")
        except Exception as e:
            print(f"🔥 Error in model_states_callback: {e}")
          

    def world_to_grid(self, pos):
        gx = int((pos[0] / self.resolution) + self.grid_center[0])
        gy = int((pos[1] / self.resolution) + self.grid_center[1])
        return gx, gy

    def grid_to_world(self, grid_pos):
        gx, gy = grid_pos
        wx = (gx - self.grid_center[0]) * self.resolution
        wy = (gy - self.grid_center[1]) * self.resolution
        return wx, wy

    def update_path(self, path):
        self.path = path
        self.path_index = 0
        self.following = False

    def start_following(self):
        self.following = True
        
    def move_one_step(self):
        self.move_along_path()

    def move_along_path(self):
        if not self.following or not self.path or len(self.path) < 2 or self.position is None:
            return

       # target = self.path[1]
        if self.path_index + 1 >= len(self.path):
            print("🎯 Path complete.")
            self.following = False
            self.stop()
            return

        target = self.path[self.path_index + 1]
        current_grid = self.world_to_grid(self.position)
        dx = target[0] - current_grid[0]
        dy = target[1] - current_grid[1]
        angle_to_target = math.atan2(dy, dx)
    
        #CHANGEDDD
        raw_yaw = quaternion_to_yaw(self.orientation)
# Apply front offset based on robot design
        current_yaw = raw_yaw + FRONT_OFFSET
        current_yaw = math.atan2(math.sin(current_yaw), math.cos(current_yaw))  # Normalize
        angle_diff = angle_to_target - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize
        
        # print(f"🔁 Aligning: angle_diff = {angle_diff:.3f} rad")
        print(f"🧭 angle_diff: {angle_diff:.3f} | yaw: {math.degrees(current_yaw):.2f}°")

        ANGLE_THRESHOLD = 0.25
        MAX_ANGULAR_SPEED = 1.0
        LINEAR_SPEED = 0.2
        ANGULAR_KP = 2.0
        
        twist = Twist()

        if abs(angle_diff) > ANGLE_THRESHOLD:
                # Count how long we've been rotating
            self.rotation_counter = getattr(self, "rotation_counter", 0) + 1
            if self.rotation_counter > 50:
                print("⚠️ Stuck rotating too long, skipping to next waypoint.")
                self.path_index += 1
                self.rotation_counter = 0
                return
            twist.linear.x = 0.0 # small nudge forward
            angular_z = ANGULAR_KP * angle_diff
            if abs(angular_z) < 0.05:
                angular_z = 0.05 * (1 if angular_z > 0 else -1)
            #twist.angular.z = ANGULAR_KP * angle_diff
            twist.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, ANGULAR_KP * angle_diff))
            #if abs(angle_diff) < 0.4:  # allow nudge if almost aligned
            #    twist.linear.x = 0.05
            #else:
            #    twist.linear.x = 0.0

        else:
            self.rotation_counter = 0  # ✅ Reset if aligned
            twist.angular.z = 0.0
            twist.linear.x = LINEAR_SPEED
            if hasattr(self, "target_heading"):
                del self.target_heading  # 🔁 Reset heading so it recalculates at next step

        self.publisher.publish(twist)
        
        # Check if we reached this waypoint
        dx = target[0] - current_grid[0]
        dy = target[1] - current_grid[1]

        distance = math.hypot(dx, dy)
        if distance < 0.6:
            self.path_index += 1
            self.stop()  # brief pause before next move
            if hasattr(self, "target_heading"):
                del self.target_heading


    def stop(self):
        self.publisher.publish(Twist())


def main():
    rclpy.init()

    x_dim, y_dim = 100, 100
    goal = (70, 80)
    view_range = 5

    new_map = OccupancyGridMap(x_dim=x_dim, y_dim=y_dim)
    gui = Animation(title="D* Lite Path Planning", width=7, height=7, margin=0,
                    x_dim=x_dim, y_dim=y_dim, start=(0, 0), goal=goal, viewing_range=view_range)
    gui.world = new_map

    slam = SLAM(map=new_map, view_range=view_range)
    follower = CmdVelController(robot_name="bot_name10", resolution=0.1, grid_center=(x_dim // 2, y_dim // 2))
    follower.grid = new_map
    follower.slam = slam  

    executor = MultiThreadedExecutor()
    executor.add_node(slam)
    executor.add_node(follower)
    threading.Thread(target=executor.spin, daemon=True).start()

    #populate_grid_from_gazebo(grid=new_map, slam=slam)

    timeout_counter = 0
    while follower.position is None:
        rclpy.spin_once(follower, timeout_sec=0.1)
        timeout_counter += 1
        if timeout_counter > 150:  # ~15 seconds
            print("❌ Timeout: Robot not found in /model_states. Exiting.")
            return
    print(f"✅ Robot found at {follower.position}")

    start_grid = follower.world_to_grid(follower.position)
    gui.set_start(start_grid)
    gui.set_position(start_grid)

    dstar = DStarLite(map=new_map, s_start=start_grid, s_goal=goal)
    initial_vertices, slam_map = slam.rescan(global_position=start_grid)
    dstar.new_edges_and_old_costs = initial_vertices
    dstar.sensed_map = slam_map
    gui.world = slam_map

   # path, g, rhs = dstar.move_and_replan(robot_position=start_grid)
    try:
        path, g, rhs = dstar.move_and_replan(robot_position=start_grid)
         # ✅ Path validity check
        if not path or len(path) < 2:
            print("❌ No valid path found or too short — skipping update.")
        else:
            follower.update_path(path)
            
    except Exception as e:
        print(f"🔥 Replanning failed: {e}")
        path = []
        dstar.k_m = 0  # optional reset
    follower.update_path(path)

    while not gui.done:
        # GUI needs to keep running
        gui.run_game(path=path)
        if follower.position is not None:
            grid_pos = follower.world_to_grid(follower.position)
            print("📍 Robot position in world:", follower.position, "→ grid:", grid_pos)
        gui.set_position(grid_pos)

           #gui.set_position(follower.world_to_grid(follower.position))
        #gui.run_game(path=path)

        if gui.space_pressed:
            follower.start_following()
            gui.space_pressed = False  # prevent triggering every loop
        follower.move_one_step()

        #follower.move_along_path()
        # Add after follower.move_along_path()

        if follower.position:
            new_pos = follower.world_to_grid(follower.position)
           # 💡 NEW: Scan for new Gazebo obstacles continuously
            updated_vertices, updated_map = slam.rescan(global_position=new_pos)
            gui.world.set_map(updated_map.get_map())
            dstar.sensed_map = updated_map
            dstar.new_edges_and_old_costs = updated_vertices
            if new_pos != start_grid:
                dstar.s_start = new_pos
                #updated_vertices, updated_map = slam.rescan(global_position=new_pos)
               # dstar.sensed_map = updated_map
               # dstar.new_edges_and_old_costs = updated_vertices
                path, g, rhs = dstar.move_and_replan(robot_position=new_pos)
                follower.update_path(path)
                follower.start_following() 
                start_grid = new_pos
    # Goal reached check
        gx, gy = goal
        nx, ny = new_pos
        if abs(gx - nx) <= 1 and abs(gy - ny) <= 1:
            print("🎯 Goal reached by real bot!")
            gui.done = True

    follower.stop()
    executor.shutdown()
    slam.destroy_node()
    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
