import numpy as np
from utils import get_movements_4n, get_movements_8n, heuristic, Vertices, Vertex
from typing import Dict, List
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates

OBSTACLE = 255
UNOCCUPIED = 0
UNSAFE = 128

class OccupancyGridMap:
    def __init__(self, x_dim, y_dim, exploration_setting='8N'):
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.map_extents = (x_dim, y_dim)
        self.occupancy_grid_map = np.zeros(self.map_extents, dtype=np.uint8)
        self.visited = {}
        self.exploration_setting = exploration_setting

    def get_map(self):
        return self.occupancy_grid_map

    def set_map(self, new_ogrid):
        self.occupancy_grid_map = new_ogrid
        
    def get_cell(self, x, y):
        if 0 <= x < self.x_dim and 0 <= y < self.y_dim:
            return self.occupancy_grid_map[y][x]
        return None  # or UNEXPLORED if you use that

    def mark_as_unsafe(self, x, y):
        if self.occupancy_grid_map[y][x] != OBSTACLE:
           self.occupancy_grid_map[y][x] = UNSAFE

    def is_unoccupied(self, pos: (int, int)) -> bool:
        (x, y) = (round(pos[0]), round(pos[1]))
        return self.occupancy_grid_map[x][y] == UNOCCUPIED

    def in_bounds(self, cell: (int, int)) -> bool:
        (x, y) = cell
        return 0 <= x < self.x_dim and 0 <= y < self.y_dim

    def filter(self, neighbors: List, avoid_obstacles: bool):
        if avoid_obstacles:
            return [node for node in neighbors if self.in_bounds(node) and self.is_unoccupied(node)]
        return [node for node in neighbors if self.in_bounds(node)]

    def succ(self, vertex: (int, int), avoid_obstacles: bool = False) -> list:
        (x, y) = vertex
        movements = get_movements_4n(x, y) if self.exploration_setting == '4N' else get_movements_8n(x, y)
        if (x + y) % 2 == 0:
            movements.reverse()
        return list(self.filter(movements, avoid_obstacles))

    def set_obstacle(self, pos: (int, int)):
        (x, y) = (round(pos[0]), round(pos[1]))
        self.occupancy_grid_map[x][y] = OBSTACLE

    def remove_obstacle(self, pos: (int, int)):
        (x, y) = (round(pos[0]), round(pos[1]))
        self.occupancy_grid_map[x][y] = UNOCCUPIED

    def local_observation(self, global_position: (int, int), view_range: int = 2) -> Dict:
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range, px + view_range + 1)
                 for y in range(py - view_range, py + view_range + 1)
                 if self.in_bounds((x, y))]
        return {node: UNOCCUPIED if self.is_unoccupied(node) else OBSTACLE for node in nodes}


class SLAM(Node):
    def __init__(self, map: OccupancyGridMap, view_range: int):
        super().__init__('slam_node')
        self.ground_truth_map = map
        self.slam_map = OccupancyGridMap(x_dim=map.x_dim, y_dim=map.y_dim)
        self.view_range = view_range
        self.model_states = None
        self.create_subscription(ModelStates, '/model_states', self.model_states_callback, 10)
        self.robot_name = "bot_name10"

    def model_states_callback(self, msg):
        self.model_states = msg
       # print("📡 Received model_states update with models:", msg.name)

    def set_ground_truth_map(self, gt_map: OccupancyGridMap):
        self.ground_truth_map = gt_map

    def c(self, u: (int, int), v: (int, int)) -> float:
        if not self.slam_map.is_unoccupied(u) or not self.slam_map.is_unoccupied(v):
            return float('inf')
        else:
            return heuristic(u, v)

    def get_dynamic_obstacles(self):
        dynamic_obs = []
        if self.model_states:
            grid_center_x = self.ground_truth_map.x_dim // 2
            grid_center_y = self.ground_truth_map.y_dim // 2
            resolution = 0.1

            for i, name in enumerate(self.model_states.name):
                if name in ["ground_plane", self.robot_name]:
                    continue
                x = self.model_states.pose[i].position.x
                y = self.model_states.pose[i].position.y
                gx = int((x / resolution) + grid_center_x)
                gy = int((y / resolution) + grid_center_y)

                print(f"[MODEL OBSTACLE] {name}: world=({x:.2f}, {y:.2f}) → grid=({gx}, {gy})")

                if 0 <= gx < self.ground_truth_map.x_dim and 0 <= gy < self.ground_truth_map.y_dim:
                    for dx in range(-1, 2):
                        for dy in range(-1, 2):
                            ox, oy = gx + dx, gy + dy
                            if 0 <= ox < self.ground_truth_map.x_dim and 0 <= oy < self.ground_truth_map.y_dim:
                                dynamic_obs.append((ox, oy))
                                print(f"[DEBUG] Models in world: {self.model_states.name}")

                              #  print(f"✅ Dynamic Obstacle: {name} → grid=({ox}, {oy})")
                else:
                    print(f"❌ Out of bounds: {name} → grid=({gx}, {gy})")
        return dynamic_obs

    def rescan(self, global_position: (int, int)):
        local_observation = self.ground_truth_map.local_observation(global_position=global_position,
                                                                    view_range=self.view_range)

        for dyn_obs in self.get_dynamic_obstacles():
            local_observation[dyn_obs] = OBSTACLE
            ox, oy = dyn_obs
            if not (0 <= ox < self.slam_map.x_dim and 0 <= oy < self.slam_map.y_dim):
                print(f"⚠️ Skipping out-of-bounds obstacle: {dyn_obs}")
                continue

        # 2. ✅ Also update slam_map used by GUI
            self.slam_map.set_obstacle(dyn_obs)

        # 3. 🔍 Optional debug print
         #    print(f"📍 Obstacle written to grid: {dyn_obs}")

        vertices = self.update_changed_edge_costs(local_grid=local_observation)
        return vertices, self.slam_map

    def update_changed_edge_costs(self, local_grid: Dict) -> Vertices:
        vertices = Vertices()
        for node, value in local_grid.items():
            old_occupied = not self.slam_map.is_unoccupied(node)
            new_occupied = value == OBSTACLE

            if old_occupied != new_occupied:  # ← Only update if changed
                v = Vertex(pos=node)
                succ = self.slam_map.succ(node)
                for u in succ:
                    v.add_edge_with_cost(succ=u, cost=self.c(u, v.pos))
                vertices.add_vertex(v)

                if new_occupied:
                    self.slam_map.set_obstacle(node)
                else:
                    self.slam_map.remove_obstacle(node)

        return vertices

