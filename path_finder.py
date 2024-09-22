import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.ndimage import zoom
import time


class PathFinder:
    def __init__(self, grid_in, goal, car_size=1, scale_factor=1):
        """
        Initialize with the grid and car size.

        Parameters:
        - grid_in: 2D numpy array (0 = free space, 1 = obstacle)
        - obstacle_map: 2D numpy array (0 = free space, 1 = obstacle)
        - goal: global goal location on grid_in
        - image_obstacles: number list (1 = stop sign, 2 = person, 3 = obstacle)
        - car_size: size of the car in grid cells (default 1)
        - scale_factor: scale down the grid by n factor
        """
        self.grid = self.scale_down_grid(grid_in, scale_factor)
        self.goal = goal
        self.car_size = car_size
        # self.add_obstacles(obstacle_map=obstacles, current_location=start, orientation=0)
        self.graph = self._grid_to_graph(self.grid)
        self.scale_factor = scale_factor
        # self.image_obstacles = image_obstacles
        self.count = 0
        self.direction = 0

    def add_image_obstacles(self, image_obstacles):
        self.image_obstacles = image_obstacles

    def add_obstacles(self, obstacle_map, current_location, orientation):
        """
        function to add obstacles from the obstacle_map on top of the global map
        :param obstacle_map: 2D numpy map to be added
        :param current_location: location of the car
        :param orientation: direction the car is point in (0 up, 1 right, 2 down, 3 left)
        :return:
        """
        scaled_map = self.scale_down_grid(obstacle_map, self.scale_factor)
        rotated_map = self.rotate_to_orientation(scaled_map, orientation)
        print("add_obstacles")
        print(self.grid.shape)
        print(rotated_map.shape)
        print(current_location)
        pada, padb, padc, padd = self.get_padding_relative_to_orientation(current_location,
                                                                          rotated_map.shape,
                                                                          self.grid.shape,
                                                                          orientation)
        print("found padding a: " + str(pada) + " b: " + str(padb) + " c: " + str(padc) + " d: " + str(padd))
        # padded_map = np.pad(rotated_map, ((padd, padb), (padc, pada)), 'constant', constant_values=0)
        padded_map = self.pad_map(pada, padb, padc, padd, rotated_map, self.grid)
        self.grid = np.add(self.grid, padded_map)
        self.graph = self._grid_to_graph(self.grid)

    def pad_map(self, pada, padb, padc, padd, map, big_map):
        big_map_shape = big_map.shape
        pad_tuple = [[0, 0], [0, 0]]
        trim_tuple = [[0, big_map_shape[0]], [0, big_map_shape[1]]]
        if pada > 0:
            pad_tuple[1][1] = pada
        # else:
        #     trim_tuple[1][1] = big_map_shape[1] - abs(pada)
        if padb > 0:
            pad_tuple[0][1] = padb
        # else:
        #     trim_tuple[0][1] = big_map_shape[0] - abs(padb)
        if padc > 0:
            pad_tuple[1][0] = padc
        else:
            trim_tuple[1][0] = abs(padc)
        if padd > 0:
            pad_tuple[0][0] = padd
        else:
            trim_tuple[0][0] = abs(padd)

        map = np.pad(map, pad_tuple, 'constant', constant_values=0)[trim_tuple[0][0]:trim_tuple[0][1],
              trim_tuple[1][0]:trim_tuple[1][1]]
        return map

    def get_padding_relative_to_orientation(self, current_location, object_map_shape, large_map_shape, orientation):
        """
        Goal is to calculate the padding parameters a, b, c, and d
        a -> padding from the bottom of the map to the large map
        b -> padding from the right side of the map to the large map
        c -> padding from the top of the map to the large map
        d -> padding from the left side of the map to the large map
        :param large_map_shape:
        :param object_map_shape:
        :param current_location:
        :param orientation:
        :return: a, b, c, d
        """
        print("stuff")
        a, b, c, d = 0, 0, 0, 0
        print("object_map" + str(object_map_shape))
        print("large_map" + str(large_map_shape))
        bigy = large_map_shape[0]
        bigx = large_map_shape[1]
        yn = bigy - current_location[0]
        xn = current_location[1]
        half_small_map = object_map_shape[0] / 2
        small_map = object_map_shape[1]
        if orientation == 0:
            a = yn
            b = bigx - xn - half_small_map
            c = bigy - yn - small_map
            d = xn - half_small_map
        elif orientation == 1:
            a = yn - half_small_map
            b = bigx - xn - small_map
            c = bigy - yn - half_small_map
            d = xn
        elif orientation == 2:
            a = yn - small_map
            b = bigx - xn - half_small_map
            c = bigy - yn
            d = xn - half_small_map
        elif orientation == 3:
            a = yn - half_small_map
            b = bigx - xn
            c = bigy - yn - half_small_map
            d = xn - small_map
        return int(a), int(b), int(c), int(d)

    def rotate_to_orientation(self, obstacle_map, orientation):
        if orientation == 0:
            return obstacle_map
        elif orientation == 1:
            return np.rot90(obstacle_map, k=3, axes=(0, 1))
        elif orientation == 2:
            return np.rot90(obstacle_map, k=2, axes=(0, 1))
        elif orientation == 3:
            return np.rot90(obstacle_map, k=1, axes=(0, 1))

    def scale_down_grid(self, grid, scale_factor):
        """
        Helper function to scale down the grid and relocate obstacles.

        Parameters:
        - grid: Original 2D numpy array
        - scale_factor: Factor by which to scale down the grid

        Returns:
        - Scaled down grid with relocated obstacles
        """
        if scale_factor == 1:
            # No scaling
            return grid

        # capture obstacle positions
        original_shape = grid.shape
        # get all obstacles
        obstacles = np.argwhere(grid == 1)

        # Scale the grid dimensions
        new_shape = (int(original_shape[0] / scale_factor), int(original_shape[1] / scale_factor))
        # New scaled-down grid with zeros
        scaled_grid = np.zeros(new_shape, dtype=int)

        # Relocate obstacles
        for (x, y) in obstacles:
            new_x = int(x / scale_factor)
            new_y = int(y / scale_factor)
            # Ensure we don't go out of bounds
            if new_x < new_shape[0] and new_y < new_shape[1]:
                scaled_grid[new_x, new_y] = 1

        return scaled_grid

    def _grid_to_graph(self, grid):
        """
        Convert the grid to networkx graph, considering the car size.

        Parameters:
        - grid: 2D numpy array

        Returns:
        - graph: A NetworkX Graph representing the grid
        """
        rows, cols = grid.shape
        G = nx.grid_2d_graph(rows, cols)

        # Remove nodes where the car can't fit due to obstacles
        obstacles = [(i, j) for i in range(rows) for j in range(cols)
                     if not self._is_position_valid(i, j)]
        G.remove_nodes_from(obstacles)
        return G

    def _is_position_valid(self, x, y):
        """
        Check if the car can occupy the position (x, y) without colliding with obstacles.
        """
        rows, cols = self.grid.shape
        if x + self.car_size > rows or y + self.car_size > cols:
            return False  # Car out of bounds
        footprint = self.grid[x:x + self.car_size, y:y + self.car_size]
        return not np.any(footprint == 1)

    def _heuristic(self, node1, node2):
        """
        Heuristic function for A* (Manhattan distance).
        """
        x1, y1 = node1
        x2, y2 = node2
        return abs(x1 - x2) + abs(y1 - y2)

    def find_path(self, start):
        """
        Find the shortest path using A* algo.

        Parameters:
        - start: tuple (x, y)
        - goal: tuple (x, y)

        Returns:
        - path: list of positions from start to goal
        """
        print(f"Start: {self.grid[start]}")
        print(f"Goal: {self.grid[self.goal]}")
        try:
            path = nx.astar_path(self.graph, start, self.goal, heuristic=self._heuristic)
            return path
        except nx.NetworkXNoPath:
            return None
        except nx.NetworkXError:
            return None
        except Exception:
            return None

    def path_to_commands(self, path):
        """
        Convert a path of tuples to movement commands.
        """
        commands = []
        if 2 in self.image_obstacles:
            commands.append("person")
        if 1 in self.image_obstacles:
            commands.append("stop")
        start_turn = True
        for i in range(1, len(path)):
            current_position = path[i - 1]
            next_position = path[i]

            delta_row = next_position[0] - current_position[0]
            delta_column = next_position[1] - current_position[1]

            # Determine the direction
            if delta_column == 1 and delta_row == 0:
                # turn needs space
                if commands and start_turn:
                    commands.pop()
                if commands and start_turn:
                    commands.pop()
                    start_turn = False
                commands.append("right")
            elif delta_column == -1 and delta_row == 0:
                # turn needs space
                if commands and start_turn:
                    commands.pop()
                if commands and start_turn:
                    commands.pop()
                    start_turn = False
                commands.append("left")

            elif delta_column == 0 and delta_row == 1:
                commands.append("backward")
                start_turn = True
            elif delta_column == 0 and delta_row == -1:
                commands.append("forward")
                start_turn = True

        return commands

    def visualize_grid(self, path=None):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(self.grid, cmap='Greys', origin='upper')

        # Plot obstacles
        obstacles = np.argwhere(self.grid == 1)
        for (x, y) in obstacles:
            rect = patches.Rectangle((y - 0.5, x - 0.5), 1, 1, linewidth=0, edgecolor=None, facecolor='black')
            ax.add_patch(rect)

        if path:
            x_coords, y_coords = zip(*path)
            ax.plot(y_coords, x_coords, color='blue', linewidth=2, label='Path')
            ax.scatter(y_coords[0], x_coords[0], color='green', s=100, label='Start')
            ax.scatter(y_coords[-1], x_coords[-1], color='red', s=100, label='Goal')
            for (x, y) in path:
                rect = patches.Rectangle((y - 0.5, x - 0.5), self.car_size, self.car_size,
                                         linewidth=1, edgecolor='blue', facecolor='blue', alpha=0.3)
                ax.add_patch(rect)

        # Legend
        legend_patches = [
            patches.Patch(color='black', label='Obstacles'),
            patches.Patch(color='blue', label='Path'),
            patches.Patch(color='green', label='Start'),
            patches.Patch(color='red', label='Goal'),
            patches.Patch(color='blue', alpha=0.3, label=f'Car Footprint (size={self.car_size})')
        ]
        ax.legend(handles=legend_patches, loc='upper right')

        ax.set_title("Grid with Obstacles and Path")
        ax.set_xlabel('Y-axis')
        ax.set_ylabel('X-axis')
        ax.grid(True, which='both', color='lightgrey', linestyle='-', linewidth=0.5)
        filename = "AStar/AStarMap" + str(self.count) + ".png"
        plt.savefig(filename)
        # plt.show()
        self.count += 1


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
