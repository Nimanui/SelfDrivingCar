import numpy as np
import matplotlib.pyplot as plt
import time
import picar_4wd as fc


# Recommendation 1: The car is in middle of grid of X-direction, and 0 y-direction
# Recommendation 2: Experiment at 5 degree increments (-60, -55, -50)

GRID_SIZE = (100, 100)
GRID = np.zeros(GRID_SIZE)
# Based on recommendation 1
CENTER_X, CENTER_Y = 49, 0
# TO BE USED FOR GRID INTERPOLATION
PREV_X, PREV_Y = None, None
# Ultrasonic Sensor Parameters
MIN_ANGLE, MAX_ANGLE = -60, 61
# Based on recommendation 2
ANGLE_STEP_SIZE = 5

# Set interpolate thresholds
MAX_DIST_DIFF = 10
MAX_ANGLE_DIFF = 10


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
    
    X_position = CENTER_X + int(dist*np.sin(theta_rad))
    Y_position = CENTER_Y + int(dist*np.cos(theta_rad))

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


def main():
    # Loop through the Minimum angle and maximum angle with the specified Angle Step Size
    for theta in range(MIN_ANGLE, MAX_ANGLE, ANGLE_STEP_SIZE):
    	# Get the reading at the angle
    	dist = fc.get_distance_at(theta)
    	print(f"Angle: {theta}, Distance: {dist}")
    	# Convert the angle to radians before passing to function
    	advanced_mapping(np.radians(theta), dist, theta)
    	time.sleep(0.1)
         
    # Image processsing
    GRID[CENTER_X, CENTER_Y] = 2
    tranformed_grid = np.rot90(GRID)
    #plt.imsave("mapping.png", tranformed_grid)

    # Reset after completing a scan
    global PREV_X, PREV_Y, PREV_DIST, PREV_THETA
    PREV_X, PREV_Y = None, None
    PREV_DIST, PREV_THETA = None, None

if __name__ == "__main__":
	try:
		main()
	finally:
		fc.stop()