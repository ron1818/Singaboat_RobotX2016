#!/usr/bin/env python

import rospy
import math
import time
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist, Quaternion
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

		self.base_frame = rospy.get_param("~base_frame", "base_link")
		self.fixed_frame = rospy.get_param("~fixed_frame", "map")
		# tf_listener
		self.tf_listener = tf.TransformListener()
		
		#remember the first pose of boat
		init_position =[self.x0, self.y0, self.yaw0]
		final_position=[self.x0+self.distance*math.cos(self.yaw0), self.y0+self.distance*math.sin(self.yaw0), self.yaw0]

		while not rospy.is_shutdown():
			constant_obj = Forward(nodename="constant_heading", is_newnode=False, target=final_position, waypoint_separation=5, is_relative=False)


	def get_tf(self, fixed_frame, base_frame):
		""" transform from base_link to map """
		trans_received = False
		while not trans_received:
			try:
				(trans, rot) = self.tf_listener.lookupTransform(fixed_frame,
																base_frame,
																rospy.Time(0))
				trans_received = True
				return (Point(*trans), Quaternion(*rot))
			except (tf.LookupException,
					tf.ConnectivityException,
					tf.ExtrapolationException):
				pass

	def odom_callback(self, msg):
		trans, rot = self.get_tf("map", "base_link")
		self.x0 = trans.x
		self.y0 = trans.y
		_, _, self.yaw0 = euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
		self.odom_received = True

if __name__ == '__main__':
	try:
	#[id,type]cruciform red
		MoveAccordingFirstOdom()

	except rospy.ROSInterruptException:
		rospy.loginfo("Task 1 Finished")