#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt

def main():

    inner = 99
    road_width = 7.5
    origin_x = -130
    origin_y = -130
    width = 520
    height = 520
    resolution = 0.5
    angle_inc = 0.001
    theta = 0
    grid = np.ones((width, height))

    for r in np.arange(inner, inner + road_width, resolution):
        theta = np.arange(0, 2*np.pi, angle_inc)
        x = r * np.cos(theta)
        y = r * np.sin(theta)

        for X, Y in zip(x, y):
            ix = int((X - origin_x) / resolution)
            iy = int((Y - origin_y) / resolution)

            if ix < 0 or iy < 0 or ix >= width or iy >= height:
                pass

            grid[iy, ix] = val

    plt.imshow(grid)
    plt.show()

if __name__ == "__main__":
    main()