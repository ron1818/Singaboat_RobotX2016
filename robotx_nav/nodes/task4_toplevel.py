#!/usr/bin/env python

""" Mission 4-SCAN THE CODE
    
    Set map border, size and origin
    Do zigzag scouting to look for light buoy
    Move toward light buoy until visible distance
    Create loiter waypoints around light buoy
	In each waypoint, do stationKeeping or holdDirection for some duration
	continue to next waypoint

    If light sequence retrieved is consistent for a number of repeats
	Set global parameter for gui result display

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
import thread
