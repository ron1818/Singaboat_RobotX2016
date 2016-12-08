#! /usr/bin/env python

import numpy as np
import math
import random
import time


def within_map(map_corners, point):
    """ determine whether a point is within a map (four corners),
    1. calculate the area of the map by sum of two triangles
    2. create 4 triangles by the point and the four corners
    3. sum the area of the 4 triangles
    4. if == area, inside, if > area, outside
    use heron's triangle equation
    """

    # map_corners = np.array[[x0,y0],[x1,y1],[x2,y2],[x3,y3]]
    # point = np.array[[xt,yt]]
    # return True or False


def distance(a, b):
    """ get the distance between two points """
    #  a = np.array[[xa,ya]], b = np.array[[xb,yb]]



if __name__ == "__main__":
    map_corners = np.array([[2,1], [1,5], [8,3], 6, 12])
    point1 = np.array([4,4])
    point2 = np.array([1,1])

    """ assignment, find whether point1 and point2 is in the map area"""

