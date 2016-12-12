#!/usr/bin/env python

""" task 4:
    -----------------
    Created by Ren Ye @ 2016-11-06
    Authors: Ren Ye, Reinaldo
    -----------------
    scan the code
"""

import rospy
import math
import time
import numpy as np
from sklearn.cluster import KMeans
from sklearn import svm
import random
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import planner_utils


class ScanTheCode(object):
    # pool = mp.Pool(2, maxtasksperchild=1)

    x_offset, y_offset = random.random() * 20 - 10, random.random() * 30 - 15
    # map_dim = [[0, 40], [0, 40]]

    exit_coordinate = [20, 40, 0]
    x0, y0, yaw0= 0, 0, 0
    totem_center = np.array([0, 0])
    totem_position = list()
    counter = 0
    MAX_DATA = 10

    markers_array = MarkerArray()

    def __init__(self, nodename="scanthecode"):
        # print("starting task 4")
        rospy.init_node(nodename, anonymous=False)
        self.rate = rospy.get_param("~rate", 1)
        rospy.Subscriber("/totem_lamp", MarkerArray, self.marker_callback, queue_size = 10)
        rospy.Subscriber("/led_sequence", String, self.led_callback, queue_size = 10)
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
        self.map_corners = np.array([[0,0], [0,40], [40,40], [40,0]])

        self.ocsvm = svm.OneClassSVM(nu=0.1, kernel="rbf", gamma=0.1)

        self.onhold_stationkeep = False
        self.onhold_moveto = False
        self.totem_find = False  # is totem find? if not, try moveto, else, stationkeep to totem
        self.led_valid = False  # from callback, if led_valid, exit

    def planner(self):
        """ return data format:
        totem_find, sk_target, moveto_target, hold_moveto, led_valid
        """
        self.stationkeep_target = list()
        self.moveto_target = list()

        if self.led_valid:  # if led is valid, exit
            self.moveto_target = self.exit_coordinate
            print "mission accomplish"
        else:  # led is not valid, continue searching
            if self.totem_find and not self.onhold_stationkeep:  # find totem
                self.stationkeep_target = self.totem_center
                self.onhold_stationkeep = True
            elif not self.totem_find and not self.onhold_moveto:  # random walk
                kwargs = {"sigma": 5}
                self.moveto_target = planner_utils.random_walk(self.map_corners, "gaussian", **kwargs)
                self.onhold_moveto = True

        return self.led_valid, self.totem_find, self.stationkeep_target, self.moveto_target

    def random_walk(self):
        """ create random walk points and more favor towards center """
        x = random.gauss(np.mean(self.map_dim[0]) + self.x_offset, 0.25 * np.ptp(self.map_dim[0]))
        y = random.gauss(np.mean(self.map_dim[1]) + self.y_offset, 0.25 * np.ptp(self.map_dim[1]))

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

    def marker_callback(self, msg):
        if len(msg.markers) > 0:
            for i in range(len(msg.markers)):
                self.totem_position.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
                self.counter += 1

        # list is full, get the totem center estimated
        if len(self.totem_position) > self.MAX_DATA:
            # do a one class svm
            self.totem_center = self.one_class_svm(self.totem_position)
            self.totem_find = True

        #visualize markers in rviz
        for i in range(len(msg.markers)):
            self.marker_pub.publish(msg.markers[i])

    def led_callback(self, msg):
        if msg.data == "found":
            self.led_valid = True
        else:
            self.led_valid = False


    def one_class_svm(self, data_list):
        """ return support vector and thus cluster center """
        data_list = np.array(data_list)
        self.ocsvm.fit(data_list)
        sv = self.ocsvm.support_vectors_
        # print sv
        # find the sv's centroid, assume only one cluster.
        return (np.mean(sv[:,0]), np.mean(sv[:,1]), 0)
        # return (np.median(sv[:,0]), np.median(sv[:,1]))

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

    def update_hold_moveto(self, onhold_moveto):
        self.onhold_moveto = onhold_moveto

    def update_hold_stationkeep(self, onhold_stationkeep):
        self.onhold_stationkeep = onhold_stationkeep


if __name__ == '__main__':
    try:
        ScanTheCode()
    except rospy.ROSInterruptException:
        rospy.loginfo("Task 4 Finished")
