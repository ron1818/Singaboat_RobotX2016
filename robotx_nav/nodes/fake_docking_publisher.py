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

class DockPublisher():
	x0, y0 = 20,10
	def __init__(self):
		rospy.init_node('dock_pub', anonymous=False)
		r = rospy.Rate(1)
		pub = rospy.Publisher("dock", MarkerArray, queue_size=50)
		#self.odom_received = False
		#rospy.Subscriber("odometry/filtered/global", Odometry, self.odom_callback, queue_size=None)
		#while not self.odom_received:
		#    r.sleep()

		count = 1
		MARKERS_MAX = 12
		self.red_x, self.red_y,self.red_z = 15, 20, 1.8
		self.green_x, self.green_y, self.green_z = 19, 20, 1.8
		self.blue_x, self.blue_y, self.blue_z = 14, 20, 1.8
		markerArray = MarkerArray()


		while not rospy.is_shutdown():
			if 0<self.x0 and self.x0<40 and 0<self.y0 and self.y0 < 22:  # the side that boat can see the shape
				if self.x0<15:
					if self.likely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("1"))
					if self.unlikely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("2"))
					if self.unlikely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("3")) 

				elif self.x0>15 and self.x0<23:  
					
					if self.likely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("1"))
					if self.likely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("2"))
					if self.likely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("3"))
					if self.unlikely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("2"))
				elif self.x0 > 23:
					if self.likely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("3"))
					if self.unlikely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("1"))
					if self.likely_spawn():
						count += 1
						if(count > MARKERS_MAX):
							markerArray.markers.pop(0)
						markerArray.markers.append(self.create_marker("2"))
		
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
		if number == "1":
			marker.id = 0  # red
			marker.type = marker.ARROW  # totem
			marker.action = marker.ADD
			marker.pose.position.x = self.red_x + self.random_noise()
			marker.pose.position.y = self.red_y + self.random_noise()
			marker.pose.position.z = self.red_z + self.random_noise()
			quaternion = quaternion_from_euler(0, 0, math.pi*3/2)
		elif number == "2":
			marker.id = 2 #blue
			marker.type = marker.SPHERE  # totem
			marker.action = marker.ADD
			marker.pose.position.x = self.blue_x + self.random_noise()
			marker.pose.position.y = self.blue_y + self.random_noise()
			marker.pose.position.z = self.blue_z + self.random_noise()
			quaternion = quaternion_from_euler(0, 0, math.pi*3/2)
		elif number == "3":
			marker.id = 1  # green
			marker.type = marker.CUBE  # totem
			marker.action = marker.ADD
			marker.pose.position.x = self.green_x + self.random_noise()
			marker.pose.position.y = self.green_y + self.random_noise()
			marker.pose.position.z = self.green_z + self.random_noise()
			quaternion = quaternion_from_euler(0, 0, math.pi*3/2)

		marker.pose.orientation.x = quaternion[0]
		marker.pose.orientation.y = quaternion[1]
		marker.pose.orientation.z = quaternion[2]
		marker.pose.orientation.w = quaternion[3] 
       

		return marker

	def random_noise(self):
		return random.random() * 6.0 - 3.0

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
		DockPublisher()
	except rospy.ROSInterruptException:
		pass

