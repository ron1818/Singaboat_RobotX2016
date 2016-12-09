#! /usr/bin/env python

import rospy
import random
import math
import numpy as np
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

class FindBreakPublisher():
	x0, y0 = 10,10
	def __init__(self):
		rospy.init_node('find_break_pub', anonymous=False)
		r = rospy.Rate(1)
		pub = rospy.Publisher("find_break", MarkerArray, queue_size=50)
		#self.odom_received = False
		#rospy.Subscriber("odometry/filtered/global", Odometry, self.odom_callback, queue_size=None)
		#while not self.odom_received:
		#    r.sleep()

		count = 1
		MARKERS_MAX = 12
		self.first_x, self.first_y = 10, 20
		self.second_x, self.second_y = 30, 20
		
		markerArray = MarkerArray()


		while not rospy.is_shutdown():
		
			if self.x0<=10 and self.x0>-10 and self.y0>0 and self.y0<40:
				if self.likely_spawn():
					count += 1
					if(count > MARKERS_MAX):
						markerArray.markers.pop(0)
					markerArray.markers.append(self.create_marker("first"))
				if self.unlikely_spawn():
					count += 1
					if(count > MARKERS_MAX):
						markerArray.markers.pop(0)
					markerArray.markers.append(self.create_marker("second"))

			elif self.x0 > 10 and self.x0<30 and self.y0>0 and self.y0<40:
				if self.likely_spawn():
					count += 1
					if(count > MARKERS_MAX):
						markerArray.markers.pop(0)
					markerArray.markers.append(self.create_marker("second"))
				if self.likely_spawn():
					count += 1
					if(count > MARKERS_MAX):
						markerArray.markers.pop(0)
					markerArray.markers.append(self.create_marker("first"))	
				
			elif self.x0 > 30 and self.x0<50 and self.y0>0 and self.y0<40:
				if self.likely_spawn():
					count += 1
					if(count > MARKERS_MAX):
						markerArray.markers.pop(0)
					markerArray.markers.append(self.create_marker("second"))
				if self.unlikely_spawn():
					count += 1
					if(count > MARKERS_MAX):
						markerArray.markers.pop(0)
					markerArray.markers.append(self.create_marker("first"))
				
		
	        	pub.publish(markerArray)

			r.sleep()

	def create_marker(self, number):
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
		if number == "first":
			marker.id = 4
			marker.type = marker.CYLINDER # totem
			marker.action = marker.ADD
			marker.pose.position.x = self.first_x + self.random_noise()
			marker.pose.position.y = self.first_y + self.random_noise()
		
			
		elif number == "second":
			marker.id = 4 #white
			marker.type = marker.CYLINDER  # totem
			marker.action = marker.ADD
			marker.pose.position.x = self.second_x + self.random_noise()
			marker.pose.position.y = self.second_y + self.random_noise()
			

			




		return marker

	def random_noise(self):
		return random.random() * 8.0 - 4.0

	def likely_spawn(self):
		return random.choice([True, False, False, False])

	def unlikely_spawn(self):
		return random.choice([False] * 15 + [True])

	def odom_callback(self, msg):
		""" call back to subscribe, get odometry data:
		pose and orientation of the current boat,
		suffix 0 is for origin """
		self.x0 = msg.pose.pose.position.x
		self.y0 = msg.pose.pose.position.y
		
		self.odom_received = True




if __name__ == "__main__":
	try:
		FindBreakPublisher()
	except rospy.ROSInterruptException:
		pass

