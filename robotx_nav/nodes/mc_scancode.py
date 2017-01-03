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
from move_base_Aiming import Aiming
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
import planner_utils


class MCScanTheCode(object):
    def __init__(self, nodename="scanthecode"):
        # print("starting task 4")
        rospy.init_node(nodename, anonymous=False)
        self.rate = rospy.get_param("~rate", 1)
        self.aiming = Aiming("aiming", is_newnode=False, target=None, radius=3, duration=100, angle_tolerance=5*math.pi/180.0, box=[0,0,0])

        self.planner()

    def planner(self, target=[-32, 26, 0], box=[-33,27,0], duration=100 ):
        self.loiter.respawn(target=target, box=box, duration=duration)

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
