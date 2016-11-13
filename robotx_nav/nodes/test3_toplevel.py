#!/usr/bin/env python

""" Mission 3-IDENTIFY SYMBOLS AND DOCK
    
    Set map border, size and origin
    Do zigzag scouting to look for dock/dock's face
    Move toward dock until certain distance
    Identify each symbols of interest, may rotate around and stay. Better if lines of symbol board can be extracted to get normal of board 
    Go to direction of symbols in sequence
	plot waypoints towards dock. Stop in dock for some time, Reverse blindly

    After all bays are visited, terminate mission and return

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
