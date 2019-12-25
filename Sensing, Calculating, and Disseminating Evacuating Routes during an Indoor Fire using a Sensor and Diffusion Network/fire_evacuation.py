import sys
sys.path.append("../")

from config import *
from lib.astar import AStar
from scipy.ndimage.morphology import grey_dilation
import numpy as np
import matplotlib.pyplot as plt
import random
from tqdm import tqdm


def simulate(
        fire_speed=2,
        occupant_speed=3,
        fire_coordinate=(36, 49),
        occupant_coordinate=(20, 15),
        visualize=False):

    # initial fire point
    fires = []
    fire = np.zeros((50, 50))
    fire[fire_coordinate[0], fire_coordinate[1]] = 1
    fires.append(fire)

    start = occupant_coordinate
    destination_doors = [(8, 0), (36, 49)]
    destination_windows = [(0, 25), (12, 49)]

    if visualize:
        fig = plt.figure(figsize=(20, 15))
        plt.ion()
        fig.show()
        fig.canvas.draw()

    f_speed = 0
    while True:
        if start in destination_doors:
            print("success!")
            return 1

        if start in destination_windows:
            print("success!")
            return 1

        # fire diffusion
        fire_area = np.zeros((50, 50))
        for fire in fires:
            fire_area += (fire*0.2)

        fmax = np.max(fire_area)
        diff = 20 - fmax
        fire_area += diff
        fire_area *= fire

        doors_accessible_count = len(destination_doors)
        evacuation_route_estimates = []
        evacuation_route_length = []
        for end in destination_doors:
            # find evacuation routes
            astar = AStar()
            astar.init_grid(50, 50, walls, fire_area, start, end)
            paths = astar.solve()
            if paths is None:
                doors_accessible_count -= 1
                continue

            evacuation_route_estimates.append(paths)
            evacuation_route_length.append(len(paths))

        windows_accessible_count = len(destination_windows)
        if doors_accessible_count <= 0:
            evacuation_route_estimates = []
            evacuation_route_length = []
            for end in destination_windows:
                # find evacuation routes
                astar = AStar()
                astar.init_grid(50, 50, walls, fire_area, start, end)
                paths = astar.solve()
                if paths is None:
                    windows_accessible_count -= 1
                    continue

                evacuation_route_estimates.append(paths)
                evacuation_route_length.append(len(paths))

        if windows_accessible_count <= 0:
            print("failed!")
            return 0

        optimal_paths = evacuation_route_estimates[evacuation_route_length.index(min(evacuation_route_length))]

        # visualize simulation
        narr = np.zeros((astar.grid_height, astar.grid_width))
        for c in astar.cells:
            if c.is_wall:
                narr[c.y, c.x] = 4
            else:
                narr[c.y, c.x] = c.fire_level

        # paths = self.get_path()
        for path in optimal_paths:
            narr[path[1], path[0]] = 7

        for x, y in destination_doors:
            narr[y, x] = 10
        for x, y in destination_windows:
            narr[y, x] = 10

        narr[start[1], start[0]] = 11

        if visualize:
            im = plt.pcolormesh(narr, cmap="jet", vmin=0, vmax=20)
            colorbar = plt.colorbar(im)
            # plt.pcolormesh(narr, vmin=0, vmax=20)
            plt.tight_layout()
            # plt.draw()
            fig.canvas.draw()
            plt.pause(0.3)
            colorbar.remove()

        # fire = grey_dilation(fire, size=(3, 3))
        # fires.append(fire)

        if fire_speed >= 1:
            for i in range(int(fire_speed)):
                fire = grey_dilation(fire, size=(3, 3))
                fires.append(fire)
        else:
            print(f_speed, int(f_speed % 1))
            f_speed += fire_speed
            if int(f_speed) >= 1:
                fire = grey_dilation(fire, size=(3, 3))
                fires.append(fire)
                f_speed -= 1

        # move occupant
        if (1 + occupant_speed) < len(optimal_paths):
            start = optimal_paths[1 + occupant_speed]
        else:
            start = optimal_paths[1]

    if visualize:
        plt.show()

if __name__ == "__main__":

    success = 0
    for i in tqdm(range(15000)):

        fire_x_coordinate = random.randrange(50)
        fire_y_coordinate = random.randrange(50)
        # fire_coordinate = (fire_x_coordinate, fire_y_coordinate)
        fire_coordinate = (2, 19)

        # occupant_coordinate = available_coord[random.randrange(len(available_coord))]
        occupant_coordinate = (38, 1)

        print(fire_coordinate, occupant_coordinate)

        result = simulate(fire_speed=1,
                          occupant_speed=1,
                          fire_coordinate=fire_coordinate,
                          occupant_coordinate=occupant_coordinate,
                          visualize=True)
        success += result

        print("Success ratio: {}".format(success / (i + 1)))
