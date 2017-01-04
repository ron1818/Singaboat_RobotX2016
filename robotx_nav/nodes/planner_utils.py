#! /usr/bin/env python

import numpy as np
import math
import random
import time
import itertools

def map_rotation(map_corners, offset, theta):
    rotated_point = [[0,0], [0,0], [0,0], [0,0]]
    count = 0
    for p in map_corners:
        op[0] = p[0] + offset[0]
        op[1] = p[1] + offset[1]
        rotated_op[count] = [math.cos(theta) * op[0] - math.sin(theta) * op[1],
                         math.sin(theta) * op[0] + math.cos(theta) * op[1]]

        count += 1
    return rotated_point


def random_walk(map_corners, offset, style, *args, **kwargs):
    """ create random walk points and avoid valid centers """
    target = None
    map_corners = np.array(map_corners)
    offset = np.array([offset])
    nap_corners += offset
    map_center = [np.mean(map_corners[:,0]), np.mean(map_corners[:,1])]

    if style == "unif":
        centers = kwargs["center"]
        threshold = kwargs["threshold"]
        # do a uniform distribution by grid search
        x_range = range(np.min(map_corners[:,0]), np.max(map_corners[:,0]), 5)
        y_range = range(np.min(map_corners[:,1]), np.max(map_corners[:,1]), 5)
        grid = list(itertools.product(x_range, y_range))
        # filter out those who is before the gate line
        while not target:
            candidate_target = random.choice(grid)
            for center in centers:  # too close to the center
                if center != []:
                    if distance(candidate_target, center[0:2]) < threshold:
                        target = None
            else:
                target = candidate_target

    elif style == "gaussian":
        sigma = kwargs["sigma"]
        while not within_map(map_corners, target):
            target = [random.gauss(map_center[0], sigma[0]),
                      random.gauss(map_center[1], sigma[1])]

    elif style == "nearby":  # near the current position
        # do a gaussian distribution with center be the boat's current position and
        sigma = kwargs["sigma"]
        x0, y0 = kwargs["current_position"]
        while not within_map(map_corners, target):
            target = [random.gauss(x0, sigma[0]), random.gauss(y0, sigma[1])]

    elif style == "near_line":  # gate data partially known, need to go around the line area
        delta_y = kwargs["delta"]
        x_range = range(np.min(self.map_corners[:,0]), np.max(self.map_corners[:,1]), 5)
        y_estimate = [self.roughline.predict(x) - delta_y * self.before_roughline_sign for x in x_range]
        choices_idx = range(len(x_range))
        candidate_target_idx = random.choice(choices_idx)
        while not target:
            if np.min(self.map_dim[1]) < y_estimate[candidate_target_idx] < np.max(self.map_dim[1]):
                # it is after the gateline
                target = [x_range(candidate_target_idx), y_estimate(candidate_target_idx)]
            else:
                target = None

    elif style == "along_line":  # gate data known, need to go to the three listener point
        line_points = kwargs["line_points"]
        while not within_map(map_corners, target):
            target = random.choice(line_points)

    elif style == "after_line":
        # do a uniform distribution by grid search
        x_range = range(np.min(self.map_dim[0]), np.max(self.map_dim[0]), 5)
        y_range = range(np.min(self.map_dim[1]), np.max(self.map_dim[1]), 5)
        grid = list(itertools.product(x_range, y_range))
        # filter out those who is before the gate line
        while not target:
            candidate_target = random.choice(grid)
            if (self.gateline.predict([candidate_target[0]]) - candidate_target[1]) * self.before_line_sign < 0:
                # it is after the gateline
                target = candidate_target
            else:
                target = None
    print target
    return [target[0], target[1], 0]

def within_map(map_corners, point):
    """ determine whether a point is within a map (four corners),
    1. calculate the area of the map by sum of two triangles
    2. create 4 triangles by the point and the four corners
    3. sum the area of the 4 triangles
    4. if  ==  area, inside, if > area, outside
    use heron's triangle equation
    * map corners must in ccw or cw sequence
    """

    if point is None:
        return False

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

