import advanced_mapping_picarx as amp
# import advanced_mapping
# import picar_4wd as fc
import path_finder as pf
import time
import numpy as np

PERSON_PAUSE = False
SPEED = 5
TIME_PAUSE = 0.1
TIME_PAUSE_TURN = 2.5
START_TURN = True
TURNING = False
ORIENTATION = 0
# 0 = "up", 1 = "right", 2 = "down", 3 = "left"

"""
ToDo: this function needs to be adjusted to move the 4WD picar which uses different functions
to the picar x
"""


# def read_command_picar4WD(px, command, start, goal):
#     global PERSON_PAUSE, TIME_PAUSE
#     if command == "stop":
#         time.sleep(1)
#         print("STOP!!!")
#     elif command == "person":
#         PERSON_PAUSE = True
#         print("PERSON!!!")
#         return start, goal
#     elif command == "backward":
#         px.set_dir_servo_angle(0)
#         px.backward(SPEED)
#         time.sleep(TIME_PAUSE)
#         goal = (goal[0], goal[1] + 1)
#     elif command == "forward":
#         px.set_dir_servo_angle(0)
#         px.forward(SPEED)
#         time.sleep(TIME_PAUSE)
#         start = (start[0], start[1] + 1)
#     elif command == "left":
#         print("left")
#         px.set_dir_servo_angle(-30)
#         px.forward(SPEED*2)
#         time.sleep(TIME_PAUSE * 4)
#         px.set_dir_servo_angle(0)
#         start = (start[0] - 1, start[1])
#     elif command == "right":
#         px.set_dir_servo_angle(30)
#         px.forward(SPEED*2)
#         time.sleep(TIME_PAUSE * 4)
#         px.set_dir_servo_angle(0)
#         start = (start[0] + 1, start[1])
#     PERSON_PAUSE = False
#     px.forward(0)
#     return start, goal
def increment_car_location_by_direction(command, current_location):
    global START_TURN, ORIENTATION, TURNING
    if TURNING:
        return current_location
    if ORIENTATION == 0:
        if command == "backward":
            current_location = (current_location[0] + 1, current_location[1])
            START_TURN = True
        elif command == "forward":
            current_location = (current_location[0] - 1, current_location[1])
            START_TURN = True
        elif command == "left":
            if START_TURN:
                current_location = (current_location[0] - 3, current_location[1] - 1)
                START_TURN = False
                TURNING = True
                ORIENTATION = 3
        elif command == "right":
            if START_TURN:
                current_location = (current_location[0] - 3, current_location[1] + 1)
                START_TURN = False
                TURNING = True
                ORIENTATION = 1
    elif ORIENTATION == 2:
        if command == "backward":
            current_location = (current_location[0] - 1, current_location[1])
            START_TURN = True
        elif command == "forward":
            current_location = (current_location[0] + 1, current_location[1])
            START_TURN = True
        elif command == "left":
            if START_TURN:
                current_location = (current_location[0] + 3, current_location[1] - 1)
                START_TURN = False
                TURNING = True
                ORIENTATION = 1
        elif command == "right":
            if START_TURN:
                current_location = (current_location[0] + 3, current_location[1] + 1)
                START_TURN = False
                TURNING = True
                ORIENTATION = 3
    elif ORIENTATION == 3:
        if command == "backward":
            current_location = (current_location[0], current_location[1] + 1)
            START_TURN = True
        elif command == "forward":
            current_location = (current_location[0], current_location[1] - 1)
            START_TURN = True
        elif command == "left":
            if START_TURN:
                current_location = (current_location[0] - 1, current_location[1] - 3)
                START_TURN = False
                TURNING = True
                ORIENTATION = 2
        elif command == "right":
            if START_TURN:
                current_location = (current_location[0] + 1, current_location[1] + 3)
                START_TURN = False
                TURNING = True
                ORIENTATION = 0
    elif ORIENTATION == 1:
        if command == "backward":
            current_location = (current_location[0], current_location[1] - 1)
            START_TURN = True
        elif command == "forward":
            current_location = (current_location[0], current_location[1] + 1)
            START_TURN = True
        elif command == "left":
            if START_TURN:
                current_location = (current_location[0] + 1, current_location[1] + 3)
                START_TURN = False
                TURNING = True
                ORIENTATION = 0
        elif command == "right":
            if START_TURN:
                current_location = (current_location[0] - 1, current_location[1] - 3)
                START_TURN = False
                TURNING = True
                ORIENTATION = 2
    return current_location

def increment_goal_by_direction(command, goal):
    global START_TURN, ORIENTATION, TURNING
    if TURNING:
        return goal
    if ORIENTATION == 0:
        if command == "backward":
            goal = (goal[0] - 1, goal[1])
            START_TURN = True
        elif command == "forward":
            goal = (goal[0] + 1, goal[1])
            START_TURN = True
        elif command == "left":
            if START_TURN:
                goal = (goal[0] + 3, goal[1] - 1)
                START_TURN = False
                TURNING = True
                ORIENTATION = 3
        elif command == "right":
            if START_TURN:
                goal = (goal[0] + 3, goal[1] + 1)
                START_TURN = False
                TURNING = True
                ORIENTATION = 1
    elif ORIENTATION == 2:
        if command == "backward":
            goal = (goal[0] + 1, goal[1])
            START_TURN = True
        elif command == "forward":
            goal = (goal[0] - 1, goal[1])
            START_TURN = True
        elif command == "left":
            if START_TURN:
                goal = (goal[0] - 3, goal[1] + 1)
                START_TURN = False
                TURNING = True
                ORIENTATION = 1
        elif command == "right":
            if START_TURN:
                goal = (goal[0] + 3, goal[1] - 1)
                START_TURN = False
                TURNING = True
                ORIENTATION = 3
    elif ORIENTATION == 3:
        if command == "backward":
            goal = (goal[0], goal[1] + 1)
            START_TURN = True
        elif command == "forward":
            goal = (goal[0], goal[1] - 1)
            START_TURN = True
        elif command == "left":
            if START_TURN:
                goal = (goal[0] + 1, goal[1] - 3)
                START_TURN = False
                TURNING = True
                ORIENTATION = 2
        elif command == "right":
            if START_TURN:
                goal = (goal[0] - 1, goal[1] + 3)
                START_TURN = False
                TURNING = True
                ORIENTATION = 0
    elif ORIENTATION == 1:
        if command == "backward":
            goal = (goal[0], goal[1] - 1)
            START_TURN = True
        elif command == "forward":
            goal = (goal[0], goal[1] + 1)
            START_TURN = True
        elif command == "left":
            if START_TURN:
                goal = (goal[0] - 1, goal[1] + 3)
                START_TURN = False
                TURNING = True
                ORIENTATION = 0
        elif command == "right":
            if START_TURN:
                goal = (goal[0] + 1, goal[1] - 3)
                START_TURN = False
                TURNING = True
                ORIENTATION = 2
    return goal

def translate_commands(command, next_command):
    if command == "forward":
        if next_command == "forward":
            return "forward"
        elif next_command == "left":
            return "left"
        elif next_command == "right":
            return "right"
    if command == "right":
        if next_command == "forward":
            return "left"
        elif next_command == "right":
            return "forward"
        elif next_command == "backward":
            return "right"
    if command == "backward":
        if next_command == "right":
            return "left"
        elif next_command == "backward":
            return "forward"
        elif next_command == "left":
            return "right"
    if command == "left":
        if next_command == "backward":
            return "left"
        elif next_command == "left":
            return "forward"
        elif next_command == "forward":
            return "right"

def read_command_picarx(px, command, start, goal):
    global PERSON_PAUSE, TIME_PAUSE, START_TURN, TURNING
    if command == "stop":
        time.sleep(1)
        print("STOP!!!")
    elif command == "person":
        PERSON_PAUSE = True
        print("PERSON!!!")
        return start, goal
    elif command == "backward":
        px.set_dir_servo_angle(0)
        px.backward(SPEED)
        time.sleep(TIME_PAUSE)
        TURNING = False
    elif command == "forward":
        px.set_dir_servo_angle(0)
        px.forward(SPEED)
        time.sleep(TIME_PAUSE)
        TURNING = False
    elif command == "left" and not TURNING:
        px.set_dir_servo_angle(-30)
        px.forward(SPEED)
        time.sleep(TIME_PAUSE_TURN)
        px.set_dir_servo_angle(0)
    elif command == "right" and not TURNING:
        px.set_dir_servo_angle(30)
        px.forward(SPEED)
        time.sleep(TIME_PAUSE_TURN)
        px.set_dir_servo_angle(0)
    start = increment_car_location_by_direction(command, start)
    PERSON_PAUSE = False
    px.forward(0)
    print("current_location: " + str(start))
    print("goal: " + str(goal))
    return start, goal


if __name__ == "__main__":
    advMap = amp.AdvancedMappingPX()
    # advMap = amp.AdvancedMapping4WD()
    try:
        scale = 5
        # start = (int(99 / scale), int(49 / scale))
        # goal = (int(20 / scale), int(49 / scale))
        start = (29, 15)
        goal = (0, 15)
        current_location = start
        orientation = 0
        index = 0
        overall_step_max = 10
        command_steps = 40
        distance = 5
        count = 0
        # grid = np.zeros((abs(start[0]) * 1.5, abs(goal[1] * 2)))
        grid = np.zeros((30 * scale, 30 * scale))
        print("Big map size" + str(grid.shape))
        pathfinder = pf.PathFinder(grid, goal, scale_factor=scale)
        while current_location != goal and index < overall_step_max:
            # rescan for a new path
            advMap.reset_grid()
            try:
                obstacle_map, image_list = advMap.sensor_loop(1 - 2 * (index % 2))
                pathfinder.add_obstacles(obstacle_map, current_location, orientation)
                pathfinder.add_image_obstacles(image_list)
                pathfinder.visualize_grid()
                path = pathfinder.find_path()
                if path:
                    pathfinder.visualize_grid(path)
                    count = pathfinder.count
                    # break
                    # follow a few steps in A*
                    commands = pathfinder.path_to_commands(path)
                    print(commands)
                    if not commands or current_location == goal:
                        print("GOAL!!!")
                    else:
                        step_count = 0
                        while step_count < command_steps:
                            command = commands.pop(0)
                            next_command = command
                            if commands:
                                next_command = commands[0]
                            real_command = translate_commands(command, next_command)
                            print(command)
                            print(commands)
                            print(real_command)
                            current_location, goal = read_command_picarx(advMap.px, real_command, current_location, goal)
                            if PERSON_PAUSE or current_location == goal:
                                step_count += command_steps
                                if current_location == goal:
                                    print("GOAL!!!")
                            step_count += 1
                else:
                    current_location, goal = read_command_picarx(advMap.px, "backward", current_location, goal)
            except Exception as e:
                current_location = goal
                index = overall_step_max
                print(e)
            finally:
                index += 1
    finally:
        advMap.px.stop()
        # fc.stop()
