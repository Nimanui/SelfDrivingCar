import numpy as np
import networkx as nx
import time


class PathFinder:
    def __init__(self, grid, image_obstacles):
        """
        Initialize with the grid.

        Parameters:
        - grid: 2D numpy array (0 = free space, 1 = obstacle)
        - image_obstacles: number list (1 = stop sign, 2 = person, 3 = obstacle)
        """
        self.grid = grid
        self.graph = self._grid_to_graph(grid)
        self.image_obstacles = image_obstacles


    def _grid_to_graph(self, grid):
        """
        Convert the grid to networkx graph.

        Parameters:
        - grid: 2D numpy array

        Returns:
        - graph: A NetworkX Graph representing the grid
        """
        rows, cols = grid.shape
        G = nx.grid_2d_graph(rows, cols)
        # obstacle nodes = 1
        obstacles = [(i, j) for i in range(rows) for j in range(cols) if grid[i, j] == 1]
        G.remove_nodes_from(obstacles)
        return G


    def _heuristic(self, node1, node2):
        """
        Heuristic function for A* (Manhattan distance).

        Parameters:
        - node1: tuple (x1, y1)
        - node2: tuple (x2, y2)

        Returns:
        - Heuristic distance between node1 and node2
        """
        x1, y1 = node1
        x2, y2 = node2
        return abs(x1 - x2) + abs(y1 - y2)

    def find_path(self, start, goal):
        """
        Find the shortest path using A* algo.

        Parameters:
        - start: tuple (x, y)
        - goal: tuple (x, y)

        Returns:
        - path: list of positions from start to goal
        """
        try:
            path = nx.astar_path(self.graph, start, goal, heuristic=self._heuristic)
            return path
        except nx.NetworkXNoPath:
            # print("No path found.")
            return None


    def path_to_commands(self, path):
        """
        Convert a path tuples to movement commands.

        Parameters:
        - path: tuples representing the path

        Returns:
        - commands: list of commands ["left", "right", "forward", "backward"]
        """
        commands = []
        if 1 in self.image_obstacles:
            commands.append("stop")
        if 2 in self.image_obstacles:
            commands.append("person")
        for i in range(1, len(path)):
            current_position = path[i - 1]
            next_position = path[i]

            delta_x = next_position[0] - current_position[0]
            delta_y = next_position[1] - current_position[1]

            # Determine the direction
            if delta_x == 1 and delta_y == 0:
                commands.append("down")
            elif delta_x == -1 and delta_y == 0:
                commands.append("up")
            elif delta_x == 0 and delta_y == 1:
                commands.append("right")
            elif delta_x == 0 and delta_y == -1:
                commands.append("left")

        return commands


'''
# Example usage:
grid = np.zeros((10, 10), dtype=int)
grid[4, 5] = 1
grid[5, 5] = 1
grid[6, 5] = 1

image_list = [1,2]

pathfinder = PathFinder(grid, image_list)
start = (0, 0)
goal = (9, 9)
path = pathfinder.find_path(start, goal)

if path:
    print("Path:", path)
    commands = pathfinder.path_to_commands(path)
    print("Commands:", commands)
else:
    print("No path found.")
'''

