#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
import os
from sklearn.cluster import KMeans
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import MarkerArray, Marker
from move_base_rotation import rotation
from move_base_forward import Forward
from move_base_force_cancel import ForceCancel
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

	red_centers=np.zeros((2, 2)) #ordered list of centers x, y
	green_centers=np.zeros((2, 2))

	red_position=np.zeros((2, 2)) #ordered list of centers x, y
	green_position=np.zeros((2, 2))

	red_counter=0
	green_counter=0
	mid_point_counter=0

	termination_displacement=50

	m=0
	d=5

	def __init__(self):
		print("starting task 1")
		rospy.init_node('task_1', anonymous=True)
		rospy.Subscriber("/fake_marker_array", MarkerArray, self.marker_callback, queue_size = 50)
		self.marker_pub= rospy.Publisher('waypoint_markers', Marker, queue_size=5)

		self.odom_received = False
		#rospy.wait_for_message("/odom", Odometry)
		#rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=50)
		rospy.wait_for_message("/odometry/filtered/global", Odometry)
		rospy.Subscriber("/odometry/filtered/global", Odometry, self.odom_callback, queue_size=50)
		while not self.odom_received:
			rospy.sleep(1)

		init_position =np.array([self.x0, self.y0, 0])

		while(self.red_counter<self.MAX_DATA and self.green_counter<self.MAX_DATA):
			#wait for data bucket to fill up
			rotation(math.pi/4)
			time.sleep(3)
			rotation(-math.pi/2)
			time.sleep(1)
		print("bucket is full")

		while not rospy.is_shutdown():
			#go to mid target
			if self.mid_point_counter > self.mid_point_counter_max:
				angle=math.atan2(m)
				move_to_goal(init_position[0]+d*math.sin(angle), init_position[1]+d*math.cos(angle))

			elif self.red_counter>self.MAX_DATA and self.green_counter>self.MAX_DATA:
				move_to_goal([mid_point_x,mid_point_y, ])

			#pass all of the pool, stop
			if self.euclid_distance(np.array([self.x0, self.y0, 0]), init_position)>self.termination_displacement:
				print("Task 1 completed")
				break

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
			self.red_centers=red_kmeans.cluster_centers_
		if(self.green_counter>self.MAX_DATA):
			green_kmeans = KMeans(n_clusters=1).fit(self.green_totem)
			self.green_centers=green_kmeans.cluster_centers_

		#updating the mid point
		if(self.red_counter%self.MAX_DATA==0 or self.green_counter%self.MAX_DATA==0)
			mid_point_x[mid_point_counter]=(self.red_centers[0]+self.green_centers[0])/2
			mid_point_y[mid_point_counter]=(self.red_centers[1]+self.green_centers[1])/2
			self.mid_point_counter=self.mid_point_counter+1
			self.m,b = np.polyfit(mid_point_x, mid_point_y, 1)

	def move_to_goal(self, goal):
		print("move to point")
		self.moveto_obj.respawn(goal, )

	def euclid_distance(self, target1, target2):
		return math.sqrt((target1[0]-target2[0])**2+(target1[1]-target2[1])**2)

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
		PassGates()
		# stage 1: gps
	except rospy.ROSInterruptException:
		rospy.loginfo("Task 1 Finished")