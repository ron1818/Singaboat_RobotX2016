#!/usr/bin/env python
#use the perpendicular 

import rospy
import math
import time
import numpy as np
import os
import tf
from sklearn.cluster import KMeans
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import euler_from_quaternion
from move_base_waypoint import MoveTo
from nav_msgs.msg import Odometry

class PassGates(object):
	MAX_DATA=30 #stash data size of marker array for clustering
	mid_point_counter_max=50

	distance=20 #distance to offset from center of gates
	replan_min=5 #replan waypoints if changes is more than this

	x0, y0, yaw0= 0, 0, 0
	markers_array=MarkerArray()

	red_totem=np.zeros((MAX_DATA, 2)) #unordered list
	green_totem=np.zeros((MAX_DATA, 2))

	red_center=np.zeros((1, 2)) #ordered list of centers x, y
	green_center=np.zeros((1, 2))

	red_position=np.zeros((1, 2)) #ordered list of centers x, y
	green_position=np.zeros((1, 2))

	red_counter=0
	green_counter=0
	mid_point_counter=0

	termination_displacement=50

	offset=5

	def __init__(self):
		print("starting task 1")
		rospy.init_node('task_1', anonymous=True)
		rospy.Subscriber("/fake_marker_array", MarkerArray, self.marker_callback, queue_size = 50)
		self.marker_pub= rospy.Publisher('waypoint_markers', Marker, queue_size=5)

		self.cmd_vel_pub = rospy.Publisher('move_base_cmd_vel', Twist, queue_size=5)

		self.base_frame = rospy.get_param("~base_frame", "base_link")
		self.fixed_frame = rospy.get_param("~fixed_frame", "map")
		# tf_listener
		self.tf_listener = tf.TransformListener()		
		self.odom_received = False
		#rospy.wait_for_message("/odom", Odometry)
		#rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=50)
		rospy.wait_for_message("/odometry/filtered/global", Odometry)
		rospy.Subscriber("/odometry/filtered/global", Odometry, self.odom_callback, queue_size=50)
		while not self.odom_received:
			rospy.sleep(1)





		init_position =np.array([self.x0, self.y0, 0])

		self.moveto_obj = MoveTo("moveto", is_newnode=False, target=None, mode=1, mode_param=1, is_relative=False)

		while(self.red_counter<self.MAX_DATA and self.green_counter<self.MAX_DATA):
			#wait for data bucket to fill up

			self.rotation(math.pi/4)
			time.sleep(3)
			self.rotation(-math.pi/2)
			time.sleep(1)
		print("bucket is full")

		while not rospy.is_shutdown():
			#main code enter here
			target=self.way_point()
			self.move_to_goal(target)

			if self.euclid_distance(init_position, np.array([self.x0, self.y0, 0]))> self.termination_displacement:
				print("task 1 completed")
				break

	def way_point(self): #go according to the perpendicular direction
		midpoint=[(self.red_center[0][0]+self.green_center[0][0])/2, (self.red_center[0][1]+self.green_center[0][1])/2]
		theta=math.atan2(math.sin(math.atan2(self.green_center[0][1]-self.red_center[0][1], self.green_center[0][0]-self.red_center[0][0])+math.pi/2), math.cos(math.atan2(self.green_center[0][1]-self.red_center[0][1], self.green_center[0][0]-self.red_center[0][0])+math.pi/2))
		alpha=math.atan2(midpoint[1]-self.y0, midpoint[0]-self.x0)
		del_angle=math.fabs(theta-alpha)

		if del_angle>math.pi/2:
			alpha=math.atan2(-math.sin(alpha), math.cos(alpha))

		angle=(theta+alpha)/2
		angle=math.atan2(math.sin(angle), math.cos(angle))

		target=[self.x0+self.offset*math.cos(angle), self.y0+self.offset*math.sin(angle), angle]

		return target

	def marker_callback(self, msg):
		if len(msg.markers)>0:
			for i in range(len(msg.markers)):
				if msg.markers[i].type == 3:
					#may append more than 1 markers
					if msg.markers[i].id == 0:
						self.red_totem[self.red_counter%self.MAX_DATA]=[msg.markers[i].pose.position.x, msg.markers[i].pose.position.y]
						self.red_counter+=1
					elif msg.markers[i].id == 1:
						self.green_totem[self.green_counter%self.MAX_DATA]=[msg.markers[i].pose.position.x, msg.markers[i].pose.position.y]
						self.green_counter+=1
				else:
					pass

		#list is full
		if (self.red_counter>self.MAX_DATA):
			red_kmeans = KMeans(n_clusters=1).fit(self.red_totem)
			self.red_center=red_kmeans.cluster_centers_
		if(self.green_counter>self.MAX_DATA):
			green_kmeans = KMeans(n_clusters=1).fit(self.green_totem)
			self.green_center=green_kmeans.cluster_centers_

	def move_to_goal(self, goal):
		print("move to point")
		self.moveto_obj.respawn(goal, )

	def euclid_distance(self, target1, target2):
		return math.sqrt((target1[0]-target2[0])**2+(target1[1]-target2[1])**2)


	def rotation(self, ang):

		rate = rospy.Rate(10)
		an_vel = 0.1
		duration = ang / an_vel
		msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, an_vel))

		rate.sleep()
		start_time = rospy.get_time()

		while not rospy.is_shutdown():
			current_time = rospy.get_time()
			if (current_time - start_time) > duration:
				self.cmd_vel_pub.publish(Twist(Vector3(0, 0.0, 0.0), Vector3(0.0, 0.0, -2 * an_vel)))
				self.cmd_vel_pub.publish(Twist())
				break
			else:
				self.cmd_vel_pub.publish(msg)
			rate.sleep()



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
		PassGates()
		# stage 1: gps
	except rospy.ROSInterruptException:
		rospy.loginfo("Task 1 Finished")