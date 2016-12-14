#!/usr/bin/env python

import rospy
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
from move_base_forward import Forward
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MoveAccordingFirstOdom(object):	
	x0, y0, yaw0= 0, 0, 0
	distance=20

	def __init__(self):
		print("Starting task 1")
		rospy.init_node('task_1', anonymous=True)

		self.odom_received = False
		#rospy.wait_for_message("/odom", Odometry)
		#rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=50)
		rospy.wait_for_message("/odometry/filtered/global", Odometry)
		rospy.Subscriber("/odometry/filtered/global", Odometry, self.odom_callback, queue_size=50)
		while not self.odom_received:
			rospy.sleep(1)
		print("odom received")
		
		#remember the first pose of boat
		init_position =[self.x0, self.y0, self.yaw0]
		final_position=[self.x0+self.distance*math.sin(self.yaw0), self.y0+self.distance*math.cos(self.yaw0), self.yaw0]

		while not rospy.is_shutdown():
			constant_obj = Forward(nodename="constant_heading", is_newnode=False, target=final_position, waypoint_separation=5, is_relative=False)

	def odom_callback(self, msg):
		""" call back to subscribe, get odometry data:
		pose and orientation of the current boat,
		suffix 0 is for origin """
		self.x0 = msg.pose.pose.position.x
		self.y0 = msg.pose.pose.position.y
		x = msg.pose.pose.orientation.x
		y = msg.pose.pose.orientation.y
		z = msg.pose.pose.orientation.z
		w = msg.pose.pose.orientation.w
		_, _, self.yaw0 = euler_from_quaternion((x, y, z, w))
		self.odom_received = True

if __name__ == '__main__':
	try:
	#[id,type]cruciform red
		MoveAccordingFirstOdom()

	except rospy.ROSInterruptException:
		rospy.loginfo("Task 1 Finished")