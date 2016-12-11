#!/usr/bin/env python

""" Mission 3-IDENTIFY SYMBOLS AND DOCK

	Set map border, size and origin
	Do zigzag scouting to look for dock/dock's face
	Move toward dock until certain distance
	Identify each symbols of interest, may rotate around and stay. Better if lines of symbol board can be extracted to get normal of board
	Go to direction of symbols in sequence
	plot waypoints towards dock. Stop in dock for some time, Reverse blindly

	After all bays are visited, terminate mission and return

	renye's approach:
	1. drive near the area by circular motion
	2. use any camera to capture the target symbol
	3. turn the bow to the target symbol
	4. turn until the ROI in both bow/left and bow/right are around the same
	5. follow the symbol by cmd_vel
	6. stop when the odom's linear_x does not increase (stuck)
	7. do a timed reverse or a odom reverse
	8. track another symbol following step 3-6

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
