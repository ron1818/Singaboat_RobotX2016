#!/usr/bin/env python

""" Mission 5-CORAL SURVEY
    
    Set map border, size and origin
    Set quadrants of interest
    
    Do zigzag scouting to respective quadrants (perpetual)
	If shape identified or duration timeout
		Shape identified->set global parameter for gui
	continue to next quadrant

   If both shapes identified or total time exceeded, terminate mission

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
