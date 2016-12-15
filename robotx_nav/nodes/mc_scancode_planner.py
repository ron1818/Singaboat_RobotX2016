#!/usr/bin/env python

""" task 4:
    -----------------
    Created by Ren Ye @ 2016-11-06
    Authors: Ren Ye, Reinaldo
    -----------------
    scan the code
"""

import rospy
import math
import time
import numpy as np
from sklearn.cluster import KMeans
from sklearn import svm
import random
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import MarkerArray, Marker
from move_base_loiter import Loiter
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import planner_utils


class MCScanTheCode(object):
    def __init__(self, nodename="scanthecode"):
        # print("starting task 4")
        rospy.init_node(nodename, anonymous=False)
        self.rate = rospy.get_param("~rate", 1)
        self.loiter = Loiter("loiter", is_newnode=False, target=None, mode=2, is_relative=True)

        self.planner()

    def planner(self):
        self.loiter.respawn(target=[5, 0, 0], polygon=6, radius=3)

        color = ["red", "red", "red"]
        for i in range(3):
            color[i] = self.mc_color()
            if i >= 1:
                while color[i] == color[i-1]:
                    color[i] = self.mc_color()

        rospy.set_param("/gui/color1", color[0])
        rospy.set_param("/gui/color2", color[1])
        rospy.set_param("/gui/color3", color[2])

    def mc_color(self):
        return random.choice(["red", "green", "blue", "yellow"])


if __name__ == '__main__':
    try:
        MCScanTheCode()
    except rospy.ROSInterruptException:
        rospy.loginfo("Task 4 Finished")
