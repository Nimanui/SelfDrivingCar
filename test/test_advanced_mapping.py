import unittest
from unittest.mock import MagicMock
import numpy as np
import sys

# Mock picar_4wd
mock_picar_4wd = MagicMock()
sys.modules['picar_4wd'] = mock_picar_4wd

import advanced_mapping

class TestAdvancedMapping(unittest.TestCase):

    def setUp(self):
        # Reset the grid and previous positions before each test
        advanced_mapping.GRID = np.zeros(advanced_mapping.GRID_SIZE)
        advanced_mapping.PREV_X = None
        advanced_mapping.PREV_Y = None

        # Reset the mock for each test
        self.mock_fc = advanced_mapping.fc = MagicMock()

    def test_single_reading(self):
        """Test a single ultrasonic reading at 0 degrees with 10 cm distance"""
        # Mock the get_distance_at method
        self.mock_fc.get_distance_at.return_value = 10

        angle = 0
        dist = self.mock_fc.get_distance_at(angle)
        theta_rad = np.radians(angle)

        advanced_mapping.advanced_mapping(theta_rad, dist)

        # Expected position in the grid
        expected_x = advanced_mapping.CENTER_X + int(dist * np.sin(theta_rad))
        expected_y = advanced_mapping.CENTER_Y + int(dist * np.cos(theta_rad))

        self.assertEqual(advanced_mapping.GRID[expected_x, expected_y], 1)

    def test_multiple_readings(self):
        """Test multiple readings at different angles and distances"""
        readings = [(0, 10), (10, 20), (-10, 15)]  # List of (angle, distance) pairs

        # Mock the get_distance_at method
        def side_effect(angle):
            for reading_angle, distance in readings:
                if angle == reading_angle:
                    return distance
            # Default distance if angle not in readings
            return 0

        self.mock_fc.get_distance_at.side_effect = side_effect

        for angle, _ in readings:
            dist = self.mock_fc.get_distance_at(angle)
            theta_rad = np.radians(angle)
            advanced_mapping.advanced_mapping(theta_rad, dist)

        # Expected positions for each reading
        for angle, dist in readings:
            theta_rad = np.radians(angle)
            expected_x = advanced_mapping.CENTER_X + int(dist * np.sin(theta_rad))
            expected_y = advanced_mapping.CENTER_Y + int(dist * np.cos(theta_rad))
            self.assertEqual(advanced_mapping.GRID[expected_x, expected_y], 1)

    def test_interpolation(self):
        """Test if interpolation is working between two points"""
        x0, y0 = 10, 10
        x1, y1 = 20, 20

        line_points = advanced_mapping.interpolate_line(x0, y0, x1, y1)

        # Check if we get more than one point
        self.assertGreater(len(line_points), 1)

        # Check if all points are within the grid bounds
        for (x, y) in line_points:
            self.assertTrue(0 <= x < advanced_mapping.GRID_SIZE[0])
            self.assertTrue(0 <= y < advanced_mapping.GRID_SIZE[1])

if __name__ == '__main__':
    unittest.main()
