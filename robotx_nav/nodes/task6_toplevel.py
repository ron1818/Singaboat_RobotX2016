#!/usr/bin/env python

import rospy
import multiprocessing as mp
import math
import time
import numpy as np
import random
import tf
from sklearn.cluster import KMeans
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import MarkerArray, Marker
from move_base_forward import Forward
from move_base_force_cancel import ForceCancel
from tf.transformations import euler_from_quaternion
from move_base_waypoint import MoveTo
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

class FindBreak(object):

	termination_displacement=60
	MAX_DATA=30
	x0,y0,yaw0=0,0,0
	x_offset, y_offset = random.random() * 10 - 5, random.random() * 10 - 5
	map_dim = [[0, 40], [0, 40]]
	totem=np.zeros((MAX_DATA,2)) #all position
	totem_centers=np.zeros((2,2)) #filtered position
	totem_position=np.zeros((2,2)) #reordered position
	counter=0
	is_collect_position_totem=1
	new_stack_totem_0=np.zeros((MAX_DATA,2))
	new_stack_totem_1=np.zeros((MAX_DATA,2))
	stash_counter_0=0
	stash_counter_1=0
	distance_between_two_totems=0
	gauss_counter=0
	tuananh_satisfied=False


	def __init__(self):
		print("starting task 6")
		rospy.init_node('task_6', anonymous=True)
		rospy.Subscriber("/filtered_marker_array", MarkerArray, self.marker_callback, queue_size = 50)
		#self.marker_pub= rospy.Publisher('waypoint_markers', Marker, queue_size=5)
		self.cmd_vel_pub = rospy.Publisher('move_base_cmd_vel', Twist, queue_size=5)

		self.base_frame = rospy.get_param("~base_frame", "base_link")
		self.fixed_frame = rospy.get_param("~fixed_frame", "map")
		# tf_listener
		self.tf_listener = tf.TransformListener()		

		self.odom_received = False
		rospy.wait_for_message("/odometry/filtered/global", Odometry)
		rospy.Subscriber("/odometry/filtered/global", Odometry, self.odom_callback, queue_size=50)
		while not self.odom_received:
			rospy.sleep(1)
		print("odom received")
		
		rospy.Subscriber("/stop_find_break", Int8, self.stop_find_callback, queue_size = 5)
		self.shooting_pub= rospy.Publisher('/start_find_break', Int8, queue_size=5)

		self.moveto_obj = MoveTo("moveto", is_newnode=False, target=None, mode=1, mode_param=1, is_relative=False)

		init_position =np.array([self.x0, self.y0, 0])

		while(self.counter<=self.MAX_DATA or self.distance_between_two_totems<5):
			#walk around to fill the bucket
			self.moveto_obj.respawn(self.random_walk(), )
			if self.counter>self.MAX_DATA:
				self.matrix_reorder()

			time.sleep(1)


		print("totem position is kinda found")
		self.matrix_reorder()
		print(self.totem_position)
		#bucket full, move to nearer
		if self.distance_from_boat(self.totem_position[0])< self.distance_from_boat(self.totem_position[1]):
			index=0
		else:
			index=1
		
		print(self.totem_position[index])

		while not rospy.is_shutdown(): #while not reach goal
			
			self.move_to_goal(self.totem_position[index])
			if self.distance_from_boat(self.totem_position[index])<2:
				break

			time.sleep(0.1)


		while not rospy.is_shutdown(): #to be changed, while tuananh not satisfy
			self.back_and_forth()
			if self.tuananh_satisfied:
				break

			time.sleep(1)

	def move_to_goal(self, goal):
		print("move to point")
		one_third_goal=[2*self.x0/3+goal[0]/3, 2*self.y0/3+goal[1]/3, math.atan2(goal[1]-self.y0, goal[0]-self.x0)]
		print(one_third_goal)
		self.moveto_obj.respawn(one_third_goal, )


	def random_walk(self):
		""" create random walk points and more favor towards center """
		if self.gauss_counter%2==1:
			x = random.gauss((self.map_dim[0][1]-self.map_dim[0][0])/4 + self.x_offset, 0.25 * np.ptp(self.map_dim[0]))
		else:
			x = random.gauss((self.map_dim[0][1]-self.map_dim[0][0])*3/4 + self.x_offset, 0.25 * np.ptp(self.map_dim[0]))
		y = random.gauss(np.mean(self.map_dim[1]) + self.y_offset, 0.25 * np.ptp(self.map_dim[1]))
		self.gauss_counter+=1
		return self.map_constrain(x, y)

	def map_constrain(self, x, y):
		""" constrain x and y within map """
		if x > np.max(self.map_dim[0]):
			x = np.max(self.map_dim[0])
		elif x < np.min(self.map_dim[0]):
			x = np.min(self.map_dim[0])
		else:
			x = x
		if y > np.max(self.map_dim[1]):
			y = np.max(self.map_dim[1])
		elif y < np.min(self.map_dim[1]):
			y = np.min(self.map_dim[1])
		else:
			y = y
		return [x, y, 0]

	def stop_find_break(self, msg):
		if msg.data==1:
			self.tuananh_satisfied=True


	def back_and_forth(self):
		while self.distance_from_boat(self.totem_position[0])>2 and not rospy.is_shutdown():
			self.move_to_goal(self.totem_position[0])
		while self.distance_from_boat(self.totem_position[1])>2 and not rospy.is_shutdown():
			self.move_to_goal(self.totem_position[1])

	def distance_from_boat(self, target):
		return math.sqrt((target[0]-self.x0)**2+(target[1]-self.y0)**2)

	def marker_callback(self, msg):#find white totem
		if len(msg.markers)>0:
			#print("hello")
			for i in range(len(msg.markers)):
				# updating totem position
				if msg.markers[i].type == 5: #no need care of the color, white color anw. horizontal totem
					self.totem[self.counter%self.MAX_DATA]=[msg.markers[i].pose.position.x, msg.markers[i].pose.position.y]
					self.counter+=1
				else:
					pass

		if self.counter > self.MAX_DATA : #list is full
			totem_kmeans=KMeans(n_clusters=2).fit(self.totem)
			self.totem_centers=totem_kmeans.cluster_centers_
			self.distance_between_two_totems=self.euclid_distance(self.totem_centers[0],self.totem_centers[1])
			if self.distance_between_two_totems<5:
				self.counter=0

			#start to have new stack of point (more accurate)
			for x in self.totem_centers:
				if self.euclid_distance(x, self.totem_position[0] )< self.euclid_distance(x, self.totem_position[1] ):
					self.new_stack_totem_0[self.stash_counter_0%self.MAX_DATA]=[x[0],x[1]]
				else:
					self.new_stack_totem_1[self.stash_counter_1%self.MAX_DATA]=[x[0],x[1]]

				if self.stash_counter_0>self.MAX_DATA: #individual stack is full, have new position
					totem_kmeans_first=KMeans(n_clusters=1).fit(self.new_stack_totem_0)
					self.totem_position[0]=totem_kmeans_first.cluster_centers_
				if self.stash_counter_1>self.MAX_DATA:
					totem_kmeans_second=KMeans(n_clusters=1).fit(self.new_stack_totem_1)
					self.totem_position[1]=totem_kmeans_second.cluster_centers_

	#to set those totems at a definite position
	def matrix_reorder(self):
		if self.totem_centers[0].dot(self.totem_centers[0].T)< self.totem_centers[1].dot(self.totem_centers[1].T):
			self.totem_position=self.totem_centers
		else:
			self.totem_position[0]=self.totem_centers[1]
			self.totem_position[1]=self.totem_centers[0]

	def euclid_distance(self, target1, target2):
		return math.sqrt((target1[0]-target2[0])**2+(target1[1]-target2[1])**2)

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
		find_break=FindBreak()
	except rospy.ROSInterruptException:
		rospy.loginfo("Task 6 Finished")
