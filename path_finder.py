import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.ndimage import zoom
import time

class PathFinder:
    def __init__(self, grid_in, image_obstacles, car_size=1, scale_factor=1, count=0):
        """
        Initialize with the grid and car size.

        Parameters:
        - grid: 2D numpy array (0 = free space, 1 = obstacle)
        - image_obstacles: number list (1 = stop sign, 2 = person, 3 = obstacle)
        - car_size: size of the car in grid cells (default 1)
        - scale_factor: scale down the grid by n factor
        """
        self.grid = self.scale_down_grid(grid_in, scale_factor)
        self.print_grid()
        self.car_size = car_size
        self.graph = self._grid_to_graph(self.grid)
        self.image_obstacles = image_obstacles
        self.count = count

    def print_grid(self):
        """debug the current matrix"""
        print('The grid visual:')
        print(
            str(self.grid)
            .replace('1', '█')
            .replace('0', '.')
            .replace(' ', '')
            .replace('[', '')
            .replace(']', '\n')
        )

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

    def find_path(self, start, goal):
        """
        Find the shortest path using A* algo.

        Parameters:
        - start: tuple (x, y)
        - goal: tuple (x, y)

        Returns:
        - path: list of positions from start to goal
        """
        print(f"Finding path from {start} to {goal}")
        try:
            path = nx.astar_path(self.graph, start, goal, heuristic=self._heuristic)
            print(f"Path found: {path}")
            return path
        except nx.NetworkXNoPath:
            print("No path found.")
            return None
        except nx.NetworkXError as e:
            print(f"NetworkX Error: {e}")
            return None
        except Exception as e:
            print(f"Unexpected error during path finding: {e}")
            return None

    def path_to_commands(self, path, current_direction='NORTH'):
        """
        Convert a path of tuples to movement commands.
        Each 'left' or 'right' turn is translated into multiple discrete turn commands.

        Parameters:
        - path: list of tuples representing the path
        - current_direction: initial direction of the car

        Returns:
        - commands: list of movement commands
        """
        commands = []

        # Handle image-based obstacles
        if 2 in self.image_obstacles:
            commands.append("person")
        if 1 in self.image_obstacles:
            commands.append("stop")

        if not path or len(path) < 2:
            return commands

        # NORTH (0), EAST (1), SOUTH (2), WEST (3)
        direction_map = {
            (-1, 0): 'NORTH',  # Moving up
            (1, 0): 'SOUTH',   # Moving down
            (0, 1): 'EAST',    # Moving right
            (0, -1): 'WEST'    # Moving left
        }

        for point in range(1, len(path)):
            current_pos = path[point - 1]
            next_pos = path[point]

            # Calculate the movement delta
            delta_row = next_pos[0] - current_pos[0]
            delta_col = next_pos[1] - current_pos[1]

            # Determine desired direction
            movement = (delta_row, delta_col)
            desired_direction = direction_map.get(movement, current_direction)

            # Add turn commands - if different direction
            if desired_direction != current_direction:
                turn = self._get_turn(current_direction, desired_direction)
                if turn:
                    if turn == 'left' or turn == 'right':
                        # approximate 90-degree turn
                        turn_steps = 3
                        commands.extend([turn] * turn_steps)
                    elif turn == 'uturn':
                        # approximate 180-degree turn
                        turn_steps = 6
                        # use 'left' for uturn
                        commands.extend(['left'] * turn_steps)
                current_direction = desired_direction

            # Add forward command
            commands.append("forward")

        return commands

    def _get_turn(self, current_direction, desired_direction):
        """
        Determine if the car needs to turn left, right, or perform a U-turn.

        Parameters:
        - current_direction: current facing direction
        - desired_direction: desired facing direction after movement

        Returns:
        - turn: 'left', 'right', or 'uturn'
        """
        directions = ['NORTH', 'EAST', 'SOUTH', 'WEST']
        current_idx = directions.index(current_direction)
        desired_idx = directions.index(desired_direction)

        # Calculate turn direction
        if (desired_idx - current_idx) % 4 == 1:
            return 'right'
        elif (current_idx - desired_idx) % 4 == 1:
            return 'left'
        elif abs(desired_idx - current_idx) == 2:
            return 'uturn'
        else:
            return None

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
