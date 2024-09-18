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
        self.pathfinder = PathFinder(self.grid)

    def test_path_exists(self):
        """Test if a path is found."""
        start = (0, 0)
        goal = (9, 9)
        path = self.pathfinder.find_path(start, goal)
        self.assertIsNotNone(path, "Error: Path should be found")
        self.assertEqual(path[0], start, "Error: Path should start at the start")
        self.assertEqual(path[-1], goal, "Error: Path should end at the goal")

    def test_no_path(self):
        """Test when there is no valid path."""
        self.grid[0, 1] = 1
        self.grid[1, 1] = 1
        self.grid[1, 0] = 1
        self.pathfinder = PathFinder(self.grid)
        start = (0, 0)
        goal = (9, 9)
        path = self.pathfinder.find_path(start, goal)
        self.assertIsNone(path, "Error: No path should be found")


    def test_path_around_obstacles(self):
        """Test path avoids obstacles."""
        start = (0, 0)
        goal = (9, 9)
        path = self.pathfinder.find_path(start, goal)
        for obstacle in [(4, 5), (5, 5), (6, 5)]:
            self.assertNotIn(obstacle, path, "Error: Path should avoid obstacles")

if __name__ == '__main__':
    unittest.main()
