import unittest
import numpy as np
from path_finder import PathFinder


class TestPathFinder(unittest.TestCase):

    def setUp(self):
        """Set up a simple grid for testing."""
        self.grid = np.zeros((10, 10), dtype=int)
        self.grid[4, 5] = 1
        self.grid[5, 5] = 1
        self.grid[6, 5] = 1
        # Example obstacle types
        self.image_obstacles = []

    def test_path_exists(self):
        """Test if a path is found."""
        pathfinder = PathFinder(self.grid, self.image_obstacles)
        start = (0, 0)
        goal = (9, 9)
        path = pathfinder.find_path(start, goal)
        self.assertIsNotNone(path, "Error: Path should be found")
        self.assertEqual(path[0], start, "Error: Path should start at the start")
        self.assertEqual(path[-1], goal, "Error: Path should end at the goal")

    def test_no_path(self):
        """Test when there is no valid path."""
        self.grid[0, 1] = 1
        self.grid[1, 1] = 1
        self.grid[1, 0] = 1
        pathfinder = PathFinder(self.grid, self.image_obstacles)
        start = (0, 0)
        goal = (9, 9)
        path = pathfinder.find_path(start, goal)
        self.assertIsNone(path, "Error: No path should be found")

    def test_commands(self):
        """Test path converts to commands."""
        pathfinder = PathFinder(self.grid, self.image_obstacles)
        start = (0, 0)
        goal = (2, 2)
        path = pathfinder.find_path(start, goal)
        commands = pathfinder.path_to_commands(path)
        self.assertEqual(commands, ['right', 'right', 'forward', 'forward'],
                         "Error: Validate the generated commands")

    def test_path_around_obstacles(self):
        """Test path avoids obstacles."""
        pathfinder = PathFinder(self.grid, self.image_obstacles)
        start = (0, 0)
        goal = (9, 9)
        path = pathfinder.find_path(start, goal)
        for obstacle in [(4, 5), (5, 5), (6, 5)]:
            self.assertNotIn(obstacle, path, "Error: Path should avoid obstacles")

    def test_commands_with_images(self):
        """Test path converts to commands."""
        image_obstacles_person = [1, 2]
        pathfinder = PathFinder(self.grid, image_obstacles_person)
        start = (0, 0)
        goal = (2, 2)
        path = pathfinder.find_path(start, goal)
        commands = pathfinder.path_to_commands(path)
        self.assertEqual(commands, ['person', 'stop', 'right', 'right', 'forward', 'forward'],
                         "Error: Validate the generated commands")

    def test_car_size1(self):
        """Test path with a larger car size."""
        pathfinder = PathFinder(self.grid, self.image_obstacles, car_size=2)
        start = (0, 0)
        goal = (8, 8)
        path = pathfinder.find_path(start, goal)
        self.assertIsNotNone(path, "Error: Path should be found for car_size=2")
        # visual the grid
        # pathfinder.visualize_grid(path)

    def test_car_size2(self):
        """Test path with a larger car size."""
        pathfinder = PathFinder(self.grid, self.image_obstacles, car_size=2)
        start = (0, 0)
        goal = (5, 8)
        path = pathfinder.find_path(start, goal)
        self.assertIsNotNone(path, "Error: Path should be found for car_size=2")
        # visual the grid
        # pathfinder.visualize_grid(path)

    def test_car_size3(self):
        """Test path with a larger car size."""

        grid = np.zeros((10, 10), dtype=int)
        grid[5, 4] = 1
        grid[5, 5] = 1
        grid[5, 6] = 1


        pathfinder = PathFinder(grid, self.image_obstacles, car_size=2)
        start = (8, 5)
        goal = (0, 5)
        path = pathfinder.find_path(start, goal)
        self.assertIsNotNone(path, "Error: Path should be found for car_size=2")
        # visual the grid
        # pathfinder.visualize_grid(path)

    def test_scale_down_grid(self):
        """Test scaling down the grid with a scale factor of 2."""

        grid = np.zeros((20, 20), dtype=int)
        grid[10, 10] = 1
        grid[11, 10] = 1

        # pathfinder with a scale factor of 2
        pathfinder = PathFinder(grid, self.image_obstacles, car_size=1, scale_factor=2)

        # Start and goal sin new smaller grid size
        start = (0, 0)
        goal = (9, 9)

        path = pathfinder.find_path(start, goal)
        self.assertIsNotNone(path, "Error: Path should be found on the scaled-down grid.")

        # visual the grid
        pathfinder.visualize_grid(path)

if __name__ == '__main__':
    unittest.main()
