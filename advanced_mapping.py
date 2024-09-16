import numpy as np
import picar_4wd as fc
import time

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

def advanced_mapping(theta_rad: int, dist: int):
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

	Returns
	=======
	None
	
	"""
	X_position = CENTER_X + int(dist*np.sin(theta_rad))
	Y_position = CENTER_Y + int(dist*np.cos(theta_rad))
	# TODO: Check if reading is within bounds of GRID
	GRID[X_position, Y_position] = 1
	# TODO: Add code for interpolation

def main():
	# Loop through the Minimum angle and maximum angle with the specified Angle Step Size
	for theta in range(MIN_ANGLE, MAX_ANGLE, ANGLE_STEP_SIZE):
		# Get the reading at the angle
		dist = fc.get_distance_at(theta)
		print(f"Angle: {theta}, Distance: {dist}")
		# Convert the angle to radians before passing to function
		advanced_mapping(np.radians(theta), dist)
		time.sleep(0.1)
		
if __name__ == "__main__":
	try:
		main()
	finally:
		fc.stop()