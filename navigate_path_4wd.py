import advanced_mapping as amp
import picar_4wd as fc
from path_finder import PathFinder
import time

PERSON_PAUSE = False
SPEED = 4
TIME_PAUSE = 0.2
TIME_PAUSE_TURN = 1.3

"""
TODO: this function needs to be adjusted to move the 4WD picar which uses different functions
to the picar x
"""
def read_command_picar4WD(command, start, goal):
     global PERSON_PAUSE, TIME_PAUSE
     if command == "stop":
         fc.stop()
         print("STOP!!!")
     elif command == "person":
         PERSON_PAUSE = True
         print("PERSON!!!")
         return start, goal
     elif command == "backward":
         fc.backward(SPEED)
         time.sleep(TIME_PAUSE)
         goal = (goal[0], goal[1] + 1)
     elif command == "forward":
         fc.forward(SPEED)
         time.sleep(TIME_PAUSE)
         start = (start[0], start[1] + 1)
     elif command == "left":
         fc.turn_left(SPEED*2)
         time.sleep(TIME_PAUSE * 2)
         start = (start[0] - 1, start[1])
     elif command == "right":
         fc.turn_right(SPEED*2)
         time.sleep(TIME_PAUSE * 2)
         start = (start[0] + 1, start[1])
     PERSON_PAUSE = False
     return start, goal


if __name__ == "__main__":
    advMap = amp.AdvancedMapping4WD()
    try:
        scale = 10
        start = (int(94/scale), int(49/scale))
        goal = (int(0/scale), int(99/scale))
        index = 0
        overall_step_max = 10
        command_steps = 3
        distance = 5
        count = 0
        while start != goal and index < overall_step_max:
            # rescan for a new path
            advMap.reset_grid()
            try:
                grid, image_list = advMap.sensor_loop(1 - 2 * (index % 2))
                pathfinder = PathFinder(grid, image_list, scale_factor=scale, count=count)
                pathfinder.visualize_grid()
                path = pathfinder.find_path(start, goal)
                pathfinder.visualize_grid(path)
                count = pathfinder.count
                # break
                # follow a few steps in A*
                if path:
                    commands = pathfinder.path_to_commands(path)
                    print('execution commands', commands)
                    if not commands:
                        print("GOAL!!!")
                    else:
                        step_count = 0
                        while step_count < command_steps:
                            command = commands.pop(0)
                            print(command)
                            print(commands)
                            start, goal = read_command_picar4WD(command, start, goal)
                            if PERSON_PAUSE or start == goal:
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
        fc.stop()
