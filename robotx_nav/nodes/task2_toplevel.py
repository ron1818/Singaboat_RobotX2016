#!/usr/bin/env python

""" Mission 2-FIND TOTEMS AND AVOID OBSTACLES
    
    Set map border, size and origin
    Set totem colors and respective loiter direction
    Do zigzag scouting, until all totems are mapped

    Go to totems of interest in sequence
	move to an offset distance from totem position
	do loiter according to direction
	remove totem from list

    After all totems visited(list of target empty) or duration time out
	terminate mission

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
