#!/usr/bin/env python

""" task 4:
    -----------------
    Created by Ren Ye @ 2016-11-06
    Authors: Ren Ye, Reinaldo
    -----------------
    scan the code
"""

import rospy
import multiprocessing as mp
import math
import time
import numpy as np
from sklearn.cluster import KMeans
from sklearn import svm
import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import MarkerArray, Marker
from move_base_forward import Forward
from move_base_waypoint import MoveTo
from move_base_loiter import Loiter
from move_base_stationkeeping import StationKeeping
from move_base_force_cancel import ForceCancel
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry


def loiter_work(target):
    print("loitering")
    loiter_obj = Loiter(nodename="loiter", target=target, radius=5, polygon=6, is_ccw=True, is_relative=False)


def stationkeeping_work(target, is_newnode):
    print("stationkeep")
    stationkeep_obj = StationKeeping(nodename="stationkeep", is_newnode = is_newnode, target=target, radius=2, duration=200)


def moveto_work(target, is_newnode):
    print("moveto")
    moveto_obj = MoveTo(nodename="moveto", is_newnode=is_newnode, target=target, is_relative=False)


def cancel_goal_work():
    print("cancels goal")
    """ asynchronously cancel goals"""
    force_cancel = ForceCancel(nodename="forcecancel", repetition=5)


class ScanTheCode(object):
    pool = mp.Pool(2, maxtasksperchild=1)

    map_dim = [[0, 40], [0, 40]]
    exit_coordinate = [20, 40]
    x0, y0, yaw0= 0, 0, 0
    totem_center = np.array([0, 0])
    MAX_DATA=30

    markers_array=MarkerArray()

    def __init__(self):
        print("starting task 4")
        rospy.init_node('task_4', anonymous=False)
        rospy.Subscriber("/totem_lamp", MarkerArray, self.marker_callback, queue_size = 50)
        self.marker_pub= rospy.Publisher('waypoint_markers', Marker, queue_size=5)

        self.ocsvm = svm.OneClassSVM(nu=0.1, kernel="rbf", gamma=0.1)

        self.odom_received = False
        #rospy.wait_for_message("/odom", Odometry)
        #rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=50)
        rospy.wait_for_message("/odometry/filtered/global", Odometry)
        rospy.Subscriber("/odometry/filtered/global", Odometry, self.odom_callback, queue_size=50)
        while not self.odom_received:
            rospy.sleep(1)
        print("odom received")
        init_position =np.array([self.x0, self.y0, 0])


        self.totem_find = False
        self.led_valid = False
        # self.first_moveto = True
        while not rospy.is_shutdown():
            # if find something, kill random moveto and do a stationkeep
            if self.totem_find and not self.led_valid:
                self.pool.apply(stationkeeping_work, args=(self.totem_center,))
            # if callback the color sequence is not valid, continue the stationkeep
            elif self.totem_find and self.led_valid:
                print "mission accomplish"
                self.pool.apply(moveto_work, args=(self.exit_coordinate,))

                break
            # if no object identified, do a random moveto, and repeat
            elif not self.totem_find and not self.led_valid:
                res = self.pool.apply(moveto_work, args=(self.random_walk(), True))
                print res.get()

            rospy.sleep(1)

        self.pool.close()
        self.pool.join()

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
