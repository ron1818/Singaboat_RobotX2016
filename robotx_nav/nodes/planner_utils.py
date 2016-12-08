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
    4. if  ==  area, inside, if > area, outside
    use heron's triangle equation
    * map corners must in ccw or cw sequence
    """

    # map_corners = np.array[[x0,y0],[x1,y1],[x2,y2],[x3,y3]]
    # point = np.array[[xt,yt]]
    # return True or False
    [[x0, y0], [x1, y1], [x2, y2], [x3, y3]] = map_corners
    [xt, yt] = point

    map_area = triangle_area([x0, y0], [x1,y1], [x2,y2])\
        + triangle_area([x0, y0], [x3,y3], [x2,y2])

    #trig 1 2 3 4
    tri1_area = triangle_area([x0, y0], [x1, y1], [xt, yt])
    tri2_area = triangle_area([x1, y1], [x2, y2], [xt, yt])
    tri3_area = triangle_area([x3, y3], [x2, y2], [xt, yt])
    tri4_area = triangle_area([x0, y0], [x3, y3], [xt, yt])
    cal_area = tri1_area + tri2_area + tri3_area + tri4_area

    if cal_area <= map_area:
        return True
    elif cal_area > map_area:
        return False


def triangle_area(A, B, C):
    """ get the triangle area """
    # ax, ay = A
    # bx, by = B
    # cx, cy = C
    a = distance(B, C)
    b = distance(A, C)
    c = distance(A, B)
    s = (a + b + c) / 2.0
    return math.sqrt(s * (s - a) * (s - b) * (s - c))


def distance(a, b):
    """ get the distance between two points """
    #  a  =  np.array[[xa,ya]], b  =  np.array[[xb,yb]]
    xa, ya = a
    xb, yb = b
    dis = math.sqrt((xa-xb)**2+(ya-yb)**2)
    return dis


if __name__  ==  "__main__":
    map_corners = np.array([[2.0,1.0], [1.0,5.0], [6.0,12.0], [8.0,3.0]])
    point1 = np.array([4.0,4.0])
    point2 = np.array([1.0,1.0])

    print within_map(map_corners, point1)
    print within_map(map_corners, point2)

    """ assignment, find whether point1 and point2 is in the map area"""

