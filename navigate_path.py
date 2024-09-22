import advanced_mapping_picarx as amp
# import advanced_mapping
# import picar_4wd as fc
import path_finder as pf
import time

PERSON_PAUSE = False
SPEED = 5
TIME_PAUSE = 0.2
TIME_PAUSE_TURN = 2.5
START_TURN = True
TURNING = False
DIRECTION = "up"

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
    global START_TURN, DIRECTION, TURNING
    if TURNING:
        return current_location
    if DIRECTION == "up":
        if command == "backward":
            current_location = (current_location[0] + 1, current_location[1])
            START_TURN = True
        elif command == "forward":
            current_location = (current_location[0] - 1, current_location[1])
            START_TURN = True
        elif command == "left":
            if START_TURN:
                current_location = (current_location[0] - 3, current_location[1] + 1)
                START_TURN = False
                TURNING = True
                DIRECTION = "left"
        elif command == "right":
            if START_TURN:
                current_location = (current_location[0] - 3, current_location[1] - 1)
                START_TURN = False
                TURNING = True
                DIRECTION = "right"
    elif DIRECTION == "down":
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
                DIRECTION = "right"
        elif command == "right":
            if START_TURN:
                current_location = (current_location[0] - 3, current_location[1] + 1)
                START_TURN = False
                TURNING = True
                DIRECTION = "left"
    elif DIRECTION == "left":
        if command == "backward":
            current_location = (current_location[0], current_location[1] - 1)
            START_TURN = True
        elif command == "forward":
            current_location = (current_location[0], current_location[1] + 1)
            START_TURN = True
        elif command == "left":
            if START_TURN:
                current_location = (current_location[0] - 1, current_location[1] + 3)
                START_TURN = False
                TURNING = True
                DIRECTION = "down"
        elif command == "right":
            if START_TURN:
                current_location = (current_location[0] + 1, current_location[1] - 3)
                START_TURN = False
                TURNING = True
                DIRECTION = "up"
    elif DIRECTION == "right":
        if command == "backward":
            current_location = (current_location[0], current_location[1] + 1)
            START_TURN = True
        elif command == "forward":
            current_location = (current_location[0], current_location[1] - 1)
            START_TURN = True
        elif command == "left":
            if START_TURN:
                current_location = (current_location[0] + 1, current_location[1] - 3)
                START_TURN = False
                TURNING = True
                DIRECTION = "up"
        elif command == "right":
            if START_TURN:
                current_location = (current_location[0] - 1, current_location[1] + 3)
                START_TURN = False
                TURNING = True
                DIRECTION = "down"
    return current_location

def increment_goal_by_direction(command, goal):
    global START_TURN, DIRECTION, TURNING
    if TURNING:
        return goal
    if DIRECTION == "up":
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
                DIRECTION = "left"
        elif command == "right":
            if START_TURN:
                goal = (goal[0] + 3, goal[1] + 1)
                START_TURN = False
                TURNING = True
                DIRECTION = "right"
    elif DIRECTION == "down":
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
                DIRECTION = "right"
        elif command == "right":
            if START_TURN:
                goal = (goal[0] + 3, goal[1] - 1)
                START_TURN = False
                TURNING = True
                DIRECTION = "left"
    elif DIRECTION == "left":
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
                DIRECTION = "down"
        elif command == "right":
            if START_TURN:
                goal = (goal[0] - 1, goal[1] + 3)
                START_TURN = False
                TURNING = True
                DIRECTION = "up"
    elif DIRECTION == "right":
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
                DIRECTION = "up"
        elif command == "right":
            if START_TURN:
                goal = (goal[0] + 1, goal[1] - 3)
                START_TURN = False
                TURNING = True
                DIRECTION = "down"
    return goal


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
    goal = increment_goal_by_direction(command, goal)
    PERSON_PAUSE = False
    px.forward(0)
    print("start: " + str(start))
    print("goal: " + str(goal))
    return start, goal


if __name__ == "__main__":
    advMap = amp.AdvancedMappingPX()
    # advMap = amp.AdvancedMapping4WD()
    try:
        scale = 4
        start = (int(99 / scale), int(49 / scale))
        goal = (int(20 / scale), int(49 / scale))
        index = 0
        overall_step_max = 10
        command_steps = 10
        distance = 5
        count = 0
        while start != goal and index < overall_step_max:
            # rescan for a new path
            advMap.reset_grid()
            try:
                grid, image_list = advMap.sensor_loop(1 - 2 * (index % 2))
                pathfinder = pf.PathFinder(grid, image_list, scale_factor=scale, count=count)
                pathfinder.visualize_grid()
                path = pathfinder.find_path(start, goal)
                if path:
                    pathfinder.visualize_grid(path)
                    count = pathfinder.count
                    # break
                    # follow a few steps in A*
                    commands = pathfinder.path_to_commands(path)
                    print(commands)
                    if not commands:
                        print("GOAL!!!")
                    else:
                        step_count = 0
                        while step_count < command_steps:
                            command = commands.pop(0)
                            print(command)
                            print(commands)
                            start, goal = read_command_picarx(advMap.px, command, start, goal)
                            if PERSON_PAUSE or start == goal:
                                step_count += command_steps
                                if start == goal:
                                    print("GOAL!!!")
                            step_count += 1
                else:
                    start, goal = read_command_picarx(advMap.px, "backward", start, goal)
            except Exception as e:
                start = goal
                index = overall_step_max
                print(e)
            finally:
                index += 1
    finally:
        advMap.px.stop()
        # fc.stop()
