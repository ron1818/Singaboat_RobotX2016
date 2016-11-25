#!/usr/bin/env python

""" task 1:
    -----------------
    Created by Ren Ye @ 2016-11-06
    Authors: Ren Ye, Reinaldo
    -----------------

    <put the descriptions from robotx.org pdf file>

    <put the algorithms in natural language, can use bullet points, best is to use markdown format>

    <if you have plan b, can put it here>

    ## example ##
    + Go to start point
    + Rotate in position to detect red_1 and green_1 buoys
    + Plot perpendicular waypoints wrt to position of red and green buoys
    + Move towards waypoints move_base_forward
    + meanwhile requesting positions of red_2 and green_2
    + 	shutdown move_base_forward, create new move_base_forward towards mid of red_2 and green_2

    <change log put here>
    ### @ 2016-11-06 ###
    + create template

    renye's approach:
    1. drive to gps waypoint
    2. slowly in place rotate # noneed
    3. detect red and green totems by any camera
    4. rotate to bow to red and green totems
    5. roi of red in bow/left and roi of green in bow/right, calculate center
    6. drive until roi vanishes from both bow cameras, detect totem from port and starboard
    7. see new roi from bow
    8. drive with 5 and 6


"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker
from math import radians, pi, sin, cos, tan, atan2
from move_base_util import MoveBaseUtil
from robotx_nav import MoveToGeo  # ?
from robotx_nav import Forward



if __name__ == '__main__':
    try:
        blahblahblah

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
