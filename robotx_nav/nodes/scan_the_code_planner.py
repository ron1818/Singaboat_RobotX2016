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


class ScanTheCode(object):
    # pool = mp.Pool(2, maxtasksperchild=1)

    map_dim = [[0, 40], [0, 40]]
    exit_coordinate = [20, 40]
    x0, y0, yaw0= 0, 0, 0
    totem_center = np.array([0, 0])
    MAX_DATA=30

    markers_array=MarkerArray()

    def __init__(self, nodename="scanthecode"):
        # print("starting task 4")
        rospy.init_node(nodename, anonymous=False)
        self.rate = rospy.get_param("~rate", 1)
        rospy.Subscriber("/totem_lamp", MarkerArray, self.marker_callback, queue_size = 50)
        self.marker_pub= rospy.Publisher('waypoint_markers', Marker, queue_size=5)

        self.ocsvm = svm.OneClassSVM(nu=0.1, kernel="rbf", gamma=0.1)

        self.hold_moveto = False # by default, let moveto to work
        self.requested_moveto = False # by default, moveto is not requested
        self.totem_find = False # is totem find? if not, try moveto, else, stationkeep to totem

    def planner(self):
        """ return data format:
        totem_find, sk_target, moveto_target, hold_moveto, led_valid
        """
        self.stationkeep_target = list()
        self.moveto_target = list()
        self.led_valid = False  # from callback, if led_valid, exit
        # if callback the color sequence is not valid, continue the stationkeep
        if self.totem_find and not self.led_valid:
            self.hold_moveto = True
            self.stationkeep_target = self.totem_center
        # if callback the color sequence is valid, complete and exit
        elif self.totem_find and self.led_valid:
            self.hold_moveto = False
            self.moveto_target = self.exit_coordinate
            print "mission accomplish"
        # if no object identified, do a random moveto, and repeat
        elif not self.totem_find and not self.led_valid:
            if not self.hold_moveto:
                self.requested_moveto = False
                self.moveto_target = self.random_walk()
            else:
                self.requested_moveto = True

        return self.totem_find, self.stationkeep_target, self.moveto_target, self.requested_moveto, self.led_valid

    def random_walk(self):
        """ create random walk points and more favor towards center """
        x = random.gauss(np.mean(self.map_dim[0]), 0.25 * np.ptp(self.map_dim[0]))
        y = random.gauss(np.mean(self.map_dim[1]), 0.25 * np.ptp(self.map_dim[1]))

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
                self.totem_position[self.counter] = [msg.markers[i].pose.position.x, msg.markers[i].pose.position.y]
                self.counter += 1

        # list is full, get the totem center estimated
        if self.totem_position > self.MAX_DATA:
            # do a one class svm
            self.totem_center = self.one_class_svm(self.totem_position)
            self.totem_find = True

        #visualize markers in rviz
        for i in range(len(msg.markers)):
            self.marker_pub.publish(msg.markers[i])

    def one_class_svm(self, data_list):
        """ return support vector and thus cluster center """
        data_list = np.array(data_list)
        self.ocsvm.fit(data_list)
        sv = self.ocsvm.support_vectors_
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



if __name__ == '__main__':
    try:
        ScanTheCode()
    except rospy.ROSInterruptException:
        rospy.loginfo("Task 4 Finished")
