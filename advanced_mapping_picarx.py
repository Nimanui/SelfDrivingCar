import numpy as np
import time
from picarx import Picarx
import matplotlib.pyplot as plt
import argparse
import sys
import path_finder as pf

import cv2
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from utils import visualize
from picamera2 import Picamera2
from vilib import Vilib


# Recommendation 1: The car is in middle of grid of X-direction, and 0 y-direction
# Recommendation 2: Experiment at 5 degree increments (-60, -55, -50)
class AdvancedMappingPX:
    def __init__(self):
        # Global variables to calculate FPS
        COUNTER, FPS = 0, 0
        START_TIME = time.time()
        self.picam2 = Picamera2()
        self.picam2.preview_configuration.main.size = (640, 480)
        self.picam2.preview_configuration.main.format = "RGB888"
        self.picam2.preview_configuration.align()
        self.picam2.configure("preview")
        self.picam2.start()

        self.GRID_SIZE = (100, 100)
        self.GRID = np.zeros(self.GRID_SIZE)
        # Based on recommendation 1
        self.CENTER_X, self.CENTER_Y = 49, 0
        # TO BE USED FOR GRID INTERPOLATION
        self.PREV_X, self.PREV_Y = None, None
        # Ultrasonic Sensor Parameters
        self.MIN_ANGLE, self.MAX_ANGLE = -60, 60
        self.MIN_ANGLE_CAM, self.MAX_ANGLE_CAM = -15, 15
        # keeping it high to start as the picarx has a bit more stuff on the servo
        self.ANGLE_STEP_SIZE = 5
        self.PREV_DIST, self.PREV_THETA = None, None

        # Set interpolate thresholds
        self.MAX_DIST_DIFF = 10
        self.MAX_ANGLE_DIFF = 10
        self.px = Picarx(ultrasonic_pins=['D0', 'D1'])

        # Visualization parameters
        self.row_size = 50  # pixels
        self.left_margin = 24  # pixels
        self.text_color = (0, 0, 0)  # black
        self.font_size = 1
        self.font_thickness = 1
        self.fps_avg_frame_count = 10

        self.detection_frame = None
        self.detection_result_list = []

        self.model = 'efficientdet_lite0.tflite'
        self.score_threshold = 0.45
        self.max_results = 5
        self.image_obstacles = []
        # Initialize the object detection model
        self.base_options = python.BaseOptions(model_asset_path=self.model)
        self.options = vision.ObjectDetectorOptions(base_options=self.base_options,
                                                    running_mode=vision.RunningMode.LIVE_STREAM,
                                                    max_results=self.max_results, score_threshold=self.score_threshold,
                                                    result_callback=self.save_result)
        self.detector = vision.ObjectDetector.create_from_options(self.options)
        self.FPS = 0
        self.COUNTER = 0
        self.START_TIME = 0

    def reset_grid(self):
        self.GRID = np.zeros(self.GRID_SIZE)
        self.image_obstacles = []

    def save_result(self, result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
        # Calculate the FPS
        if self.COUNTER % self.fps_avg_frame_count == 0:
            self.FPS = self.fps_avg_frame_count / (time.time() - self.START_TIME)
            self.START_TIME = time.time()

        self.detection_result_list.append(result)
        self.COUNTER += 1

    def interpolate_line(self, x0, y0, x1, y1):
        """Interpolates the line between two points in the grid."""

        points = []

        delta_x = abs(x1 - x0)
        delta_y = abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = delta_x - delta_y

        while x0 != x1 or y0 != y1:
            points.append((x0, y0))
            error2 = 2 * error
            if error2 > -delta_y:
                error -= delta_y
                x0 += step_x
            if error2 < delta_x:
                error += delta_x
                y0 += step_y

        points.append((x1, y1))

        return points

    def advanced_mapping(self, theta_rad: int, dist: int, theta_deg: int):
        """
        Function to convert sensor readings into Cartesian
        coordiate grid.	Performs interpolation of grid cells
        between readings.

        Parameters
        ==========
        theta_rad : int
            The angle (in radians) used by the ultrasonic
            sensor for the reading.

        dist : int
            The distance (cm) to the detected object at
            the input angle.

        theta_deg : int
            The angle (in degrees) used by the ultrasonic
            sensor for the reading.

        Returns
        =======
        None

        """

        X_position = self.CENTER_X + int(dist * np.sin(theta_rad))
        Y_position = self.CENTER_Y + int(dist * np.cos(theta_rad))

        print(f"Updating grid at X={X_position}, Y={Y_position}, Distance={dist}, Theta={theta_rad}")

        # New: Check if reading is within bounds of GRID
        if 0 <= X_position < self.GRID_SIZE[0] and 0 <= Y_position < self.GRID_SIZE[1]:
            self.GRID[X_position, Y_position] = 1
            # New: Add code for interpolation
            if self.PREV_X is not None and self.PREV_Y is not None:
                dist_diff = abs(dist - self.PREV_DIST)
                angle_diff = abs(theta_deg - self.PREV_THETA)

                if dist_diff <= self.MAX_DIST_DIFF and angle_diff <= self.MAX_ANGLE_DIFF:
                    line_points = self.interpolate_line(self.PREV_X, self.PREV_Y, X_position, Y_position)
                    for (x, y) in line_points:
                        if 0 <= x < self.GRID_SIZE[0] and 0 <= y < self.GRID_SIZE[1]:
                            self.GRID[x, y] = 1

            self.PREV_X, self.PREV_Y = X_position, Y_position
            self.PREV_DIST, self.PREV_THETA = dist, theta_deg

    def detection_results(self, fps_text):
        if self.detection_result_list:
            detected_list = self.detection_result_list[0].detections
            for detected in detected_list:
                category = detected.categories
                category_found = category[0].category_name
                if category_found == "stop sign":
                    print("stop")
                    print(detected)
                    self.image_obstacles.append(1)
                elif category_found == "person" or category_found == "potted plant":
                    print("stop, person")
                    print(detected)
                    self.image_obstacles.append(2)
                    if category_found == "potted plant":
                        print("plant staff fun")
                elif category_found != "":
                    print(detected)
                    self.image_obstacles.append(3)
                else:
                    print(fps_text)
            # print(detection_result_list)
            self.detection_result_list.clear()

    def sensor_loop(self, direction):
        """
        Parameters:
        - direction: left to right or right to left? 1 for left to right, -1 for right to left
        """
        # Loop through the Minimum angle and maximum angle with the specified Angle Step Size
        for theta in range(direction * self.MIN_ANGLE, direction * (self.MAX_ANGLE + 1),
                           direction * self.ANGLE_STEP_SIZE):
            # Get the reading at the angle
            self.px.set_cam_pan_angle(theta)
            dist = round(self.px.ultrasonic.read(), 2)
            print(f"Angle: {theta}, Distance: {dist}")
            # Convert the angle to radians before passing to function
            self.advanced_mapping(np.radians(theta), dist, theta)
            if self.MIN_ANGLE_CAM < theta < self.MAX_ANGLE_CAM:
                im = self.picam2.capture_array()
                #    success, image = cap.read()
                image = cv2.resize(im, (640, 480))
                image = cv2.flip(image, -1)

                # Convert the image from BGR to RGB as required by the TFLite model.
                rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

                # Run object detection using the model.
                self.detector.detect_async(mp_image, time.time_ns() // 1_000_000)
                # Show the FPS
                fps_text = 'FPS = {:.1f}'.format(self.FPS)
                self.detection_results(fps_text)
            time.sleep(0.1)

        # Image processsing
        self.GRID[self.CENTER_X, self.CENTER_Y] = 2
        transformed_grid = np.rot90(self.GRID)
        # if direction == 1:
        #     plt.imsave("mapping.png", transformed_grid)
        # else:
        #     plt.imsave("mapping_reverse.png", transformed_grid)
        map_name = "map/mapping" + str(self.COUNTER) + ".png"
        plt.imsave(map_name, transformed_grid)
        # Reset after completing a scan
        self.PREV_X, self.PREV_Y = None, None
        self.PREV_DIST, self.PREV_THETA = None, None
        print(self.image_obstacles)
        return transformed_grid, self.image_obstacles

#
# if __name__ == "__main__":
#     advMap = AdvancedMapping()
#     try:
#         start = (0, 0)
#         goal = (9, 9)
#         grid, image_list = advMap.sensor_loop(1)
#         pathfinder = pf.PathFinder(grid, image_list)
#         index = 0
#         path = pathfinder.find_path(start, goal)
#         commands = []
#         # if path:
#         #     commands = pathfinder.path_to_commands(path)
#         # steps = 3
#         # distance = 5
#         # while len(commands) > 0:
#         #     for i in range(0, steps, 1):
#         #         command = commands.pop()
#         #
#         #     sensor_loop(-1)
#     finally:
#         advMap.px.stop()
