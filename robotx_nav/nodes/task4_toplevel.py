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

# def constant_heading(goal):
#     print("const_heading")
#     constant_obj = Forward(nodename="constant_heading", target=goal, waypoint_separation=5, is_relative=False)
#


def loiter_work(target):
    print("loitering")
    loiter_obj = Loiter(nodename="loiter", target=target, radius=5, polygon=6, is_ccw=True, is_relative=False)


def stationkeeping_work(target):
    print("stationkeep")
    stationkeep_obj = StationKeeping(nodename="stationkeep", target=target, radius=2, duration=200)


def moveto_work(target):
    print("moveto")
    moveto_obj = MoveTo(nodename="moveto", target=target, is_relative=False)


def cancel_goal():
    print("cancels goal")
    """ asynchronously cancel goals"""
    force_cancel = ForceCancel(nodename="forcecancel", repetition=5)


class ScanTheCode(object):
    pool = mp.Pool(2)

    map_dim = [[0, 0], [-40, 40]]
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


        while not rospy.is_shutdown():
            # if no object identified, do a random moveto, and repeat

            # if find something, kill random moveto and do a stationkeep

            # if callback the color sequence is not valid, continue the stationkeep



        self.pool.close()
        self.pool.join()


    def planner(self):
        """ if cannot get totem center, do a random walk toward the center
        so need a target for moveto
        if find a totem center, do a station keeping to that point, make the time long
        """

    def marker_callback(self, msg):
        if len(msg.markers)>0:
        for i in range(len(msg.markers)):
            self.totem_position[self.counter] = [msg.markers[i].pose.position.x, msg.markers[i].pose.position.y]
            self.counter += 1

        # list is full, get the totem center estimated
        if (self.totem_position>self.MAX_DATA):
            # do a one class svm
            self.totem_center = self.one_class_svm(self.totem_position)

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
        WaypointPublisher()
        # stage 1: gps
    except rospy.ROSInterruptException:
        rospy.loginfo("Task 1 Finished")
