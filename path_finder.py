import numpy as np
import networkx as nx

class PathFinder:
    def __init__(self, grid):
        """
        Initialize with the grid.

        Parameters:
        - grid: 2D numpy array (0 = free space, 1 = obstacle)
        """
        self.grid = grid
        self.graph = self._grid_to_graph(grid)

    def _grid_to_graph(self, grid):
        """
        Convert the grid to a NetworkX graph.

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

    def heuristic(self, node1, node2):
        """
        Heuristic function for A* (Manhattan distance).

        Parameters:
        - node1: Tuple (x1, y1)
        - node2: Tuple (x2, y2)

        Returns:
        - Heuristic distance between node1 and node2
        """
        x1, y1 = node1
        x2, y2 = node2
        return abs(x1 - x2) + abs(y1 - y2)

    def find_path(self, start, goal):
        """
        Find the shortest path using A* algorithm.

        Parameters:
        - start: Tuple (x, y)
        - goal: Tuple (x, y)

        Returns:
        - path: List of positions from start to goal
        """
        try:
            path = nx.astar_path(self.graph, start, goal, heuristic=self.heuristic)
            return path
        except nx.NetworkXNoPath:
            print("No path found.")
            return None

"""
Example usage:
grid = np.zeros((10, 10), dtype=int)
grid[4, 5] = 1
grid[5, 5] = 1
grid[6, 5] = 1

pathfinder = PathFinder(grid)
start = (0, 0)
goal = (9, 9)
path = pathfinder.find_path(start, goal)

if path:
    print("Path:", path)
else:
    print("No path found.")
"""