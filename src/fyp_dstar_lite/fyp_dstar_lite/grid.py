import numpy as np
from fyp_dstar_lite.utils import get_movements_4n, get_movements_8n, heuristic, Vertices, Vertex
from typing import Dict, List

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
            return self.occupancy_grid_map[x][y]
        return None

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
