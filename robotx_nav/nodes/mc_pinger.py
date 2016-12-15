#!/usr/bin/env python

""" Mission 8-

"""
import rospy
import multiprocessing as mp
import math
import time
import numpy as np
import os
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from move_base_waypoint import MoveTo
from move_base_loiter import Loiter
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class Pinger(object):

    x0, y0, yaw0= 0, 0, 0
    d=5
    k=10
    loiter_radius=8

    def __init__(self):
        print("starting task 8 hydrophone")
        rospy.init_node('task_8', anonymous=True)

        self.loiter_obj = Loiter("loiter", is_newnode=False, target=None, radius=5, polygon=4, mode=1, mode_param=1, is_relative=False)
        self.moveto_obj = MoveTo("moveto", is_newnode=False, target=None, is_relative=False)

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

        init_position=[self.x0, self.y0, self.yaw0]



        target=[self.x0+self.d*math.cos(self.yaw0), self.y0+self.d*math.sin(self.yaw0), self.yaw0]
        black_buoy=[self.x0+self.k*math.cos(self.yaw0), self.y0+self.k*math.sin(self.yaw0), self.yaw0]
        self.moveto_obj.respawn(target, )

        #loiter around station until symbol's face seen

        self.loiter_obj.respawn(black_buoy, self.loiter_radius, )


        self.moveto_obj.respawn(init_position, )

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
        Pinger()

    except rospy.ROSInterruptException:
        rospy.loginfo("Task hydrophone pinger Finished")
