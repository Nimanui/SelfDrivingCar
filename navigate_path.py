import advanced_mapping_picarx as amp
# import advanced_mapping
# import picar_4wd as fc
import path_finder as pf
import time

PERSON_PAUSE = False
SPEED = 5
TIME_PAUSE = 0.5


def read_command_picarx(px, command, start, goal):
    global PERSON_PAUSE, TIME_PAUSE
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
        goal = (goal[0], goal[1] + 1)
    elif command == "forward":
        px.set_dir_servo_angle(0)
        px.forward(SPEED)
        time.sleep(TIME_PAUSE)
        start = (start[0], start[1] + 1)
    elif command == "left":
        print("left")
        px.set_dir_servo_angle(-30)
        px.forward(SPEED*2)
        time.sleep(TIME_PAUSE * 4)
        px.set_dir_servo_angle(0)
        start = (start[0] - 1, start[1])
    elif command == "right":
        px.set_dir_servo_angle(30)
        px.forward(SPEED*2)
        time.sleep(TIME_PAUSE * 4)
        px.set_dir_servo_angle(0)
        start = (start[0] + 1, start[1])
    PERSON_PAUSE = False
    px.forward(0)
    return start, goal


if __name__ == "__main__":
    advMap = amp.AdvancedMapping()
    try:
        start = (2, 0)
        goal = (2, 3)
        # grid, image_list = advMap.sensor_loop(1)
        # pathfinder = pf.PathFinder(grid, image_list)
        index = 0
        overall_step_max = 10
        # path = pathfinder.find_path(start, goal)
        # commands = []
        # if path:
        #     commands = pathfinder.path_to_commands(path)
        command_steps = 3
        distance = 5
        while start != goal and index < overall_step_max:
            # rescan for a new path
            advMap.reset_grid()
            try:
                grid, image_list = advMap.sensor_loop(1 - 2 * (index % 2))
                pathfinder = pf.PathFinder(grid, image_list)
                path = pathfinder.find_path(start, goal)
                # follow a few steps in A*
                if path:
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
                                print("PERSON PAUSE!!!")
                                step_count += command_steps
                                if start == goal:
                                    print("GOAL!!!")
                            step_count += 1

                else:
                    start = goal
                    index = overall_step_max
            except Exception as e:
                start = goal
                index = overall_step_max
                print(e)
            finally:
                index += 1
    finally:
        advMap.px.stop()
        # fc.stop()
