import pygame
import time
from grid import OccupancyGridMap
from typing import List

# Define some colors
BLACK = (0, 0, 0)  # BLACK
UNOCCUPIED = (255, 255, 255)  # WHITE
GOAL = (0, 255, 0)  # GREEN
START = (255, 0, 0)  # RED
GRAY1 = (145, 145, 102)  # GRAY1
OBSTACLE = (77, 77, 51)  # GRAY2
LOCAL_GRID = (0, 0, 80)  # BLUE
UNSAFE = (255, 165, 0) # ORANGE

colors = {
    0: UNOCCUPIED,
    1: GOAL,
    255: OBSTACLE,
    128: UNSAFE
}


class Animation:
    def __init__(self,
                 title="D* Lite Path Planning",
                 width=10,
                 height=10,
                 margin=0,
                 x_dim=60,
                 y_dim=40,
                 start=(0, 0),
                 goal=(50, 50),
                 viewing_range=3):

        self.width = width
        self.height = height
        self.margin = margin
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.start = start

        self.autoplay = False
        self.step_timer = time.time()
        self.step_delay = 1

        self.current = start
        self.observation = {"pos": None, "type": None}
        self.goal = goal
        self.space_pressed = False
        self.viewing_range = viewing_range

        pygame.init()
        window_size = [(width + margin) * y_dim + margin,
                       (height + margin) * x_dim + margin]
        self.screen = pygame.display.set_mode(window_size)
        self.world = OccupancyGridMap(x_dim=x_dim,
                                      y_dim=y_dim,
                                      exploration_setting='8N')
        pygame.display.set_caption(title)
        pygame.font.SysFont('Comic Sans MS', 36)
        self.done = False
        self.clock = pygame.time.Clock()

    def get_position(self):
        return self.current

    def set_position(self, pos: (int, int)):
        self.current = pos

    def get_goal(self):
        return self.goal

    def set_goal(self, goal: (int, int)):
        self.goal = goal
        self.space_pressed = False

    def set_start(self, start: (int, int)):
        self.start = start

    def display_path(self, path=None):
        if path is not None:
            for step in path:
                step_center = [round(step[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                               round(step[0] * (self.height + self.margin) + self.height / 2) + self.margin]
                pygame.draw.circle(self.screen, START, step_center, round(self.width / 2) - 2)

    def display_obs(self, observations=None):
        if observations is not None:
            for o in observations:
                pygame.draw.rect(self.screen, GRAY1, [(self.margin + self.width) * o[1] + self.margin,
                                                      (self.margin + self.height) * o[0] + self.margin,
                                                      self.width,
                                                      self.height])

    def run_game(self, path=None):
        if path is None:
            path = []

        grid_cell = None
        self.cont = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("quit")
                self.done = True

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                self.space_pressed = True
                print("▶ Path following triggered by SPACE")
                self.space_pressed = True
                print("▶ Path following triggered by SPACE")
                self.autoplay = True
                print("▶ Autoplay started")

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_BACKSPACE:
                print("backspace automates the press space")
                self.cont = not self.cont

            elif pygame.mouse.get_pressed()[0]:
                (col, row) = pygame.mouse.get_pos()
                x = row // (self.height + self.margin)
                y = col // (self.width + self.margin)
                grid_cell = (x, y)
                if self.world.is_unoccupied(grid_cell):
                    self.world.set_obstacle(grid_cell)
                    self.observation = {"pos": grid_cell, "type": OBSTACLE}

            elif pygame.mouse.get_pressed()[2]:
                (col, row) = pygame.mouse.get_pos()
                x = row // (self.height + self.margin)
                y = col // (self.width + self.margin)
                grid_cell = (x, y)
                if not self.world.is_unoccupied(grid_cell):
                    print("grid cell: {}".format(grid_cell))
                    self.world.remove_obstacle(grid_cell)
                    self.observation = {"pos": grid_cell, "type": UNOCCUPIED}

        if self.autoplay and path and self.current != self.goal:
            if time.time() - self.step_timer >= self.step_delay:
               # if self.current in path:
                   # idx = path.index(self.current)
                   # if idx + 1 < len(path):
                       # self.current = path[idx + 1]
                self.step_timer = time.time()

        self.screen.fill(BLACK)

        for row in range(self.x_dim):
            for column in range(self.y_dim):
                pygame.draw.rect(self.screen, colors[self.world.occupancy_grid_map[row][column]],
                                 [(self.margin + self.width) * column + self.margin,
                                  (self.margin + self.height) * row + self.margin,
                                  self.width,
                                  self.height])

        self.display_path(path=path)
        pygame.draw.rect(self.screen, GOAL, [(self.margin + self.width) * self.goal[1] + self.margin,
                                             (self.margin + self.height) * self.goal[0] + self.margin,
                                             self.width,
                                             self.height])

        robot_center = [round(self.current[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                        round(self.current[0] * (self.height + self.margin) + self.height / 2) + self.margin]
        pygame.draw.circle(self.screen, START, robot_center, round(self.width / 2) - 2)

        pygame.draw.rect(self.screen, LOCAL_GRID,
                         [robot_center[0] - self.viewing_range * (self.height + self.margin),
                          robot_center[1] - self.viewing_range * (self.width + self.margin),
                          2 * self.viewing_range * (self.height + self.margin),
                          2 * self.viewing_range * (self.width + self.margin)], 2)
                          
        pygame.event.pump()
        self.clock.tick(20)
        if self.current == self.goal:
            print("🎯 Goal reached! Exiting...")
            self.done = True

        pygame.display.flip()
    pygame.quit()
