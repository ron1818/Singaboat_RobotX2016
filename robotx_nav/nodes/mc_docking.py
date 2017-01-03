#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import euler_from_quaternion
from move_base_waypoint import MoveTo
from move_base_reverse import Reversing
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker

class Docking(object):

	x0,y0,yaw0=0,0,0
	min_distance=3
	shape_type=list()
	shape_position=np.zeros((2, 3))
	current_shape=0


	def __init__(self, shape_type):
		print("starting docking task 3")
		rospy.init_node('task_3', anonymous=False)
		self.shape_type=shape_type
		rospy.wait_for_message("/filtered_marker_array", MarkerArray)
		rospy.Subscriber("/filtered_marker_array", MarkerArray, self.marker_callback, queue_size = 50)
		#rospy.Subscriber("/dock", MarkerArray, self.marker_callback, queue_size = 50)
		self.base_frame = rospy.get_param("~base_frame", "base_link")
		self.fixed_frame = rospy.get_param("~fixed_frame", "map")
		self.tf_listener = tf.TransformListener()

		self.odom_received = False
		rospy.wait_for_message("/odometry/filtered/global", Odometry)
		rospy.Subscriber("/odometry/filtered/global", Odometry, self.odom_callback, queue_size=50)
		while not self.odom_received:
			rospy.sleep(1)
		print("odom received")


		self.moveto_obj = MoveTo("moveto", is_newnode=False, target=None, mode=1, mode_param=1, is_relative=False)
		self.reverse_obj= Reversing("reverse", is_newnode=False, mode="timed", speed=-2, duration=15, distance=5)




		#for i in range(len(shape_type)):
			#self.shape_position.append([0, 0, 0])

		self.init_position=[self.x0, self.y0, self.yaw0]
		rospy.sleep(1)
		#if wanted item is found, go to that position until distance<3 or 4 m
		while not rospy.is_shutdown() and self.current_shape<len(self.shape_type):
			if(self.shape_position[self.current_shape][0]!=0):
				self.move_to_goal(self.shape_position[self.current_shape])

			if (self.distance_from_boat(self.shape_position[self.current_shape])<self.min_distance):
				self.reverse()
				self.moveto_obj.respawn(self.init_position, )
				self.current_shape+=1

			time.sleep(1)


	def move_to_goal(self, goal):
		print("move to point")
		one_third_goal=[2*self.x0/3+goal[0]/3, 2*self.y0/3+goal[1]/3, math.atan2(goal[1]-self.y0, goal[0]-self.x0)]
		print(one_third_goal)
		self.moveto_obj.respawn(one_third_goal, )


	def reverse(self):
		print("reversing")
		self.reverse_obj.respawn()


	def distance_from_boat(self, target):
		return math.sqrt((target[0]-self.x0)**2+(target[1]-self.y0)**2)

	def marker_callback(self, msg):#find white totem

		for i in range(len(msg.markers)):
			# get current shape's position
                        print self.shape_type
                        try:
                            if msg.markers[i].type == self.shape_type[self.current_shape]: #check if shape is what we want
                                    theta=math.atan2(msg.markers[i].pose.position.x-self.x0, msg.markers[i].pose.position.y-self.y0)
                                    self.shape_position[self.current_shape]=[msg.markers[i].pose.position.x, msg.markers[i].pose.position.y, theta]
                                    print(self.shape_position)
                        except:
                            pass


	def euclid_distance(self, target1, target2):
		return math.sqrt((target1[0]-target2[0])**2+(target1[1]-target2[1])**2)

	def get_tf(self, fixed_frame, base_frame):
		""" transform from base_link to map """
		trans_received = False
		while not trans_received:
			try:
				(trans, rot) = self.tf_listener.lookupTransform(fixed_frame, base_frame, rospy.Time(0))
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
		docking=Docking([2, 1])
	except rospy.ROSInterruptException:
		rospy.loginfo("Task 3 Finished")
