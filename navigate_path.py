import advanced_mapping_picarx as amp
#import advanced_mapping
#import picar_4wd as fc
import path_finder as pf

#
# def read_command_picarx(px, command):
#


if __name__ == "__main__":
    advMap = amp.AdvancedMapping()
    try:
        start = (0, 0)
        goal = (9, 9)
        grid, image_list = advMap.sensor_loop(1)
        pathfinder = pf.PathFinder(grid, image_list)
        index = 0
        path = pathfinder.find_path(start, goal)
        commands = []
        # if path:
        #     commands = pathfinder.path_to_commands(path)
        # steps = 3
        # distance = 5
        # while len(commands) > 0:
        #     for i in range(0, steps, 1):
        #         command = commands.pop()
        #
        #     sensor_loop(-1)
    finally:
        advMap.px.stop()
        #fc.stop()