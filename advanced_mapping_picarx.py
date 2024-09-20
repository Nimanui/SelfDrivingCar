import numpy as np
import time
from picarx import Picarx
import argparse
import sys

import cv2
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from utils import visualize
from picamera2 import Picamera2

# Recommendation 1: The car is in middle of grid of X-direction, and 0 y-direction
# Recommendation 2: Experiment at 5 degree increments (-60, -55, -50)

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

GRID_SIZE = (100, 100)
GRID = np.zeros(GRID_SIZE)
# Based on recommendation 1
CENTER_X, CENTER_Y = 49, 0
# TO BE USED FOR GRID INTERPOLATION
PREV_X, PREV_Y = None, None
# Ultrasonic Sensor Parameters
MIN_ANGLE, MAX_ANGLE = -60, 61
# keeping it high to start as the picarx has a bit more stuff on the servo
ANGLE_STEP_SIZE = 10

# Set interpolate thresholds
MAX_DIST_DIFF = 10
MAX_ANGLE_DIFF = 10
px = Picarx(ultrasonic_pins=['D0', 'D1'])

# Visualization parameters
row_size = 50  # pixels
left_margin = 24  # pixels
text_color = (0, 0, 0)  # black
font_size = 1
font_thickness = 1
fps_avg_frame_count = 10

detection_frame = None
detection_result_list = []

model = 'efficientdet_lite0.tflite'
score_threshold = 0.25
max_results = 5


def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
    global FPS, COUNTER, START_TIME

    # Calculate the FPS
    if COUNTER % fps_avg_frame_count == 0:
        FPS = fps_avg_frame_count / (time.time() - START_TIME)
        START_TIME = time.time()

    detection_result_list.append(result)
    COUNTER += 1


# Initialize the object detection model
base_options = python.BaseOptions(model_asset_path=model)
options = vision.ObjectDetectorOptions(base_options=base_options,
                                       running_mode=vision.RunningMode.LIVE_STREAM,
                                       max_results=max_results, score_threshold=score_threshold,
                                       result_callback=save_result)
detector = vision.ObjectDetector.create_from_options(options)


def interpolate_line(x0, y0, x1, y1):
    """Interpolates the line between two points in the grid."""

    points = []

    delta_x = abs(x1 - x0)
    delta_y = abs(y1 - y0)
    step_x = 1 if x0 < x1 else -1
    step_y = 1 if y0 < y1 else -1
    error = delta_x - delta_y

    while (x0 != x1 or y0 != y1):
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


def advanced_mapping(theta_rad: int, dist: int, theta_deg: int):
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
    global PREV_X, PREV_Y, PREV_DIST, PREV_THETA

    X_position = CENTER_X + int(dist * np.sin(theta_rad))
    Y_position = CENTER_Y + int(dist * np.cos(theta_rad))

    print(f"Updating grid at X={X_position}, Y={Y_position}, Distance={dist}, Theta={theta_rad}")

    # New: Check if reading is within bounds of GRID
    if 0 <= X_position < GRID_SIZE[0] and 0 <= Y_position < GRID_SIZE[1]:
        GRID[X_position, Y_position] = 1
        # New: Add code for interpolation
        if PREV_X is not None and PREV_Y is not None:
            dist_diff = abs(dist - PREV_DIST)
            angle_diff = abs(theta_deg - PREV_THETA)

            if dist_diff <= MAX_DIST_DIFF and angle_diff <= MAX_ANGLE_DIFF:
                line_points = interpolate_line(PREV_X, PREV_Y, X_position, Y_position)
                for (x, y) in line_points:
                    if 0 <= x < GRID_SIZE[0] and 0 <= y < GRID_SIZE[1]:
                        GRID[x, y] = 1

        PREV_X, PREV_Y = X_position, Y_position
        PREV_DIST, PREV_THETA = dist, theta_deg


def detection_results(fps_text):
    if detection_result_list:
        found_list = []
        detected_list = detection_result_list[0].detections
        for detected in detected_list:
            category = detected.categories
            if category[0].category_name == "stop sign":
                print("stop")
                print(detected)
                found_list.append("stop")
            elif category[0].category_name == "person":
                print("stop, person")
                print(detected)
                found_list.append("person")
            elif category[0].category_name != "":
                print(detected)
                found_list.append("obstacle")
            else:
                print(fps_text)
        # print(detection_result_list)
        detection_result_list.clear()
        return found_list

def main():
    image_obstacles = []
    # Loop through the Minimum angle and maximum angle with the specified Angle Step Size
    for theta in range(MIN_ANGLE, MAX_ANGLE, ANGLE_STEP_SIZE):
        # Get the reading at the angle
        px.set_cam_pan_angle(theta)
        dist = round(px.ultrasonic.read(), 2)
        print(f"Angle: {theta}, Distance: {dist}")
        # Convert the angle to radians before passing to function
        advanced_mapping(np.radians(theta), dist, theta)
        im = picam2.capture_array()
        #    success, image = cap.read()
        image = cv2.resize(im, (640, 480))
        image = cv2.flip(image, -1)

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        # Run object detection using the model.
        image_obstacles.append(detector.detect_async(mp_image, time.time_ns() // 1_000_000))
        # Show the FPS
        fps_text = 'FPS = {:.1f}'.format(FPS)
        detection_results(fps_text)
        time.sleep(0.1)

    # Reset after completing a scan
    global PREV_X, PREV_Y, PREV_DIST, PREV_THETA
    PREV_X, PREV_Y = None, None
    PREV_DIST, PREV_THETA = None, None


if __name__ == "__main__":
    try:
        main()
    finally:
        px.stop()
