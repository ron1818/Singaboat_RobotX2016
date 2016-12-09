#! /usr/bin/env python

import rospy
import random
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

"""
type of marker:
ARROW = 0   --> triangle
CUBE=1      --> cruciform
SPHERE=2    --> circle
CYLINDER=3  --> totem
LINE_STRIP=4--> rectangle
id: color:
RED=0
GREEN=1
BLUE=2
BLACK=3
WHITE=4
YELLOW=5
ORANGE=6
"""

class ShootingPublisher():
	x0, y0 = 0,0
	def __init__(self):
		rospy.init_node('shoot_pub', anonymous=False)
		r = rospy.Rate(1)
		pub = rospy.Publisher("shoot", MarkerArray, queue_size=50)
		self.odom_received = False
		rospy.Subscriber("odometry/filtered/global", Odometry, self.odom_callback, queue_size=None)
		while not self.odom_received:
		    r.sleep()

		count = 1
		MARKERS_MAX = 12
		self.e_x, self.e_y,self.e_z = 21, 20, 1.5  #east
		self.n_x, self.n_y, self.n_z = 20, 21, 1.5  # north
		self.w_x, self.w_y, self.w_z = 19, 20, 1.5 #west
		self.s_x, self.s_y, self.s_z = 20, 19, 1.5 #south
		markerArray = MarkerArray()


		while not rospy.is_shutdown():
			if self.x0**2+self.y0**2 <= 400:  #the radius is 20 for camera to see
				if self.x0>self.y0 and self.x0<-self.y0 and self.y0<-1.14: #south
					if self.likely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("s"))

				elif self.y0<self.x0 and self.x0>-self.y0 and self.x0 >1.14: #east  
					
					if self.likely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("e"))
				elif self.x0<self.y0 and -self.x0<self.y0 and self.y0 > 1.14: #North
					if self.likely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("n"))
				elif self.x0<self.y0 and self.x0<-self.y0 and self.x0< -1.14: #west
					if self.likely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("w"))
					
		
	        	pub.publish(markerArray)

			r.sleep()

	def create_marker(self, orientation):
		marker = Marker()
		marker.header.frame_id = "/map"
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		# create start red totem
		marker.header.stamp = rospy.Time.now()
		
		marker.pose.orientation.w = 1.0    



		marker.pose.position.z = 1.8
		if orientation == "e":
			marker.id = 1  # green
			marker.type = marker.SPHERE  # CIRCLE
			marker.action = marker.ADD
			marker.pose.position.x = self.e_x + self.random_noise()
			marker.pose.position.y = self.e_y + self.random_noise()
			marker.pose.position.z = self.e_z + self.random_noise()
			quaternion = quaternion_from_euler(0, 0, 0)
		elif orientation == "n":
			marker.id = 2 #blue
			marker.type = marker.ARROW  # TRIANGLE
			marker.action = marker.ADD
			marker.pose.position.x = self.n_x + self.random_noise()
			marker.pose.position.y = self.n_y + self.random_noise()
			marker.pose.position.z = self.n_z + self.random_noise()
			quaternion = quaternion_from_euler(0, 0, math.pi/2)
		elif orientation == "w":
			marker.id = 1  # green
			marker.type = marker.LINE_STRIP  # rectangle
			marker.action = marker.ADD
			marker.pose.position.x = self.w_x + self.random_noise()
			marker.pose.position.y = self.w_y + self.random_noise()
			marker.pose.position.z = self.w_z + self.random_noise()
			quaternion = quaternion_from_euler(0, 0, math.pi)
		elif orientation == "s":
			marker.id = 0  # red
			marker.type = marker.CUBE  # cruciform
			marker.action = marker.ADD
			marker.pose.position.x = self.w_x + self.random_noise()
			marker.pose.position.y = self.w_y + self.random_noise()
			marker.pose.position.z = self.w_z + self.random_noise()
			quaternion = quaternion_from_euler(0, 0, math.pi*3/2)

		marker.pose.orientation.x = quaternion[0]
		marker.pose.orientation.y = quaternion[1]
		marker.pose.orientation.z = quaternion[2]
		marker.pose.orientation.w = quaternion[3]       

		return marker

	def random_noise(self):
		return random.random() * 1.0 - 1.0

	def likely_spawn(self):
		return random.choice([True, False, False, False])

	def unlikely_spawn(self):
		return random.choice([False] * 15 + [True])

	def odom_callback(self, msg):
		""" call back to subscribe, get odometry data:
		pose and orientation of the current boat,
		suffix 0 is for origin """
		self.x0 = msg.pose.pose.position.x-20
		self.y0 = msg.pose.pose.position.y-20
		self.odom_received = True




if __name__ == "__main__":
	try:
		ShootingPublisher()
	except rospy.ROSInterruptException:
		pass

