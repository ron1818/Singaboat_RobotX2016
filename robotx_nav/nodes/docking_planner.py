#!/usr/bin/env python
import random
import itertools
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String
import numpy as np
import math
from sklearn.cluster import KMeans, DBSCAN
from sklearn import svm
from sklearn.linear_model import LinearRegression
from move_base_loiter import Loiter
from move_base_waypoint import MoveTo
from move_base_reverse import Reversing
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf



class Docking(object):
    """ find coordinate of totem for task1 """
    docker_list, first_dock_list, second_dock_list, facing_list = list(), list(), list(), list()
    MAX_LENS = 20 # actually 21
    map_dim = [[0, 40], [0, 40]]
    docker_center, first_dock_center, second_dock_center = list(), list(), list()
    clearance = [3, 1]
    facing, aiming = 0, 0
    x0, y0, yaw0 = 0, 0, 0
    initial_position = list()

    def __init__(self, nodename="docking_planner", assigned=[[0,0], [1,1]]):
        rospy.init_node(nodename)
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param("~rate", 1)
        self.assigned = assigned
        self.tf_listener = tf.TransformListener()

        self.kmeans = KMeans(n_clusters=2)
        self.ocsvm = svm.OneClassSVM(nu=0.1, kernel="rbf", gamma=0.1)
        self.moveto = MoveTo("moveto", is_newnode=False, target=None, mode=1, mode_param=1, is_relative=False)
        self.loiter = Loiter("loiter", is_newnode=False, target=None, is_relative=False)
        self.timed_reverse = Reversing("reverse", is_newnode=False, mode="timed")

        # Subscribe to marker array publisher
        self.odom_received = False
        rospy.Subscriber("odometry/filtered/global", Odometry, self.odom_callback, queue_size=None)
        while not self.odom_received:
            pass
        rospy.Subscriber("filtered_marker_array", MarkerArray, self.markerarray_callback, queue_size=10)

        self.docker_find = False
        self.symbols_find = False
        self.fist_entry_path = list()  # later need to fill in
        self.second_entry_path = list()  # later need to fill in
        # while not self.x0:
        self.initial_position = [self.x0, self.y0]

        self.planner()

    def planner(self):
        """ find docker object, go around the docker, find the three symbols,
        find the center and entry point of the docks, go for dock1, reverse,
        go for dock2, reverse, exit """
        r = rospy.Rate(self.rate)

        while not self.docker_find:
            print "gauss"
            self.moveto.respawn(self.random_walk("before_line"))
            r.sleep()
        else:  # find the docker object
            # need to identify the symbols
            print "docker object find"
            while not self.symbols_find:
                print "loiter around"
                print self.docker_center
                self.loiter.respawn(self.docker_center)
                r.sleep()
            else:
                print "symbols find"
                # find the entry point who has pinger
                # self.identify_symbol()
                print "enter first"
                self.moveto.respawn(self.first_entry_path[0] + [0])
                self.moveto.respawn(self.first_entry_path[1] + [0])
                self.timed_reverse.respawn()
                print "enter second"
                self.moveto.respawn(self.second_entry_path[0] + [0])
                self.moveto.respawn(self.second_entry_path[1] + [0])
                self.timed_reverse.respawn()
                print "exit"
                # self.moveto.respawn(self.exit_point)
                print "complete"

    def odom_callback(self, msg):
        """ call back to subscribe, get odometry data:
        pose and orientation of the current boat,
        suffix 0 is for origin """
        # self.x0 = msg.pose.pose.position.x
        # self.y0 = msg.pose.pose.position.y
        trans, rot = self.get_tf("map", "base_link")
        self.x0 = trans.x
        self.y0 = trans.y
        _, _, self.yaw0 = euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
        self.odom_received = True

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


    def markerarray_callback(self, msg):
        """ determine docker and symbols """
        # accumulate data
        for i in range(len(msg.markers)):
            # whatever in the marker array, collect it
            if len(self.docker_list) > 10 * self.MAX_LENS:
                self.docker_list.pop(0)
            self.docker_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
            # identify symbols
            if msg.markers[i].id == self.assigned[0][0] and msg.markers[i].type == self.assigned[0][1]: # first assigned color and shape
                if len(self.first_dock_list) > self.MAX_LENS:
                    self.first_dock_list.pop(0)
                _, _, yaw = euler_from_quaternion((msg.markers[i].pose.orientation.x,
                                                   msg.markers[i].pose.orientation.y,
                                                   msg.markers[i].pose.orientation.z,
                                                   msg.markers[i].pose.orientation.w))
                self.first_dock_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])  # , yaw])
                if len(self.facing_list) > 2 * self.MAX_LENS:
                    self.facing_list.pop(0)
                self.facing_list.append([yaw])
            elif msg.markers[i].id == self.assigned[1][0] and msg.markers[i].type == self.assigned[1][1]: # second assigned color and shape
                if len(self.second_dock_list) > self.MAX_LENS:
                    self.second_dock_list.pop(0)
                _, _, yaw = euler_from_quaternion((msg.markers[i].pose.orientation.x,
                                                   msg.markers[i].pose.orientation.y,
                                                   msg.markers[i].pose.orientation.z,
                                                   msg.markers[i].pose.orientation.w))
                self.second_dock_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])  # , yaw])
                if len(self.facing_list) > 2 * self.MAX_LENS:
                    self.facing_list.pop(0)
                self.facing_list.append([yaw])
        self.find_docker_position()  # find the docker position
        self.identify_symbol()

    def find_docker_position(self):
        """ use ocsvm to find the position of the docker"""
        if len(self.docker_list) >= 10 * self.MAX_LENS:
            self.docker_center = self.one_class_svm(self.docker_list)
            self.docker_find = True  # for planner

    def identify_symbol(self):
        """ use ocsvm to find the position of 1st, and 2nd docking point,
        average out the yaw to get the facing """

        if len(self.first_dock_list) > self.MAX_LENS and len(self.second_dock_list) > self.MAX_LENS:
            # determine the centers
            self.first_dock_center = self.one_class_svm(self.first_dock_list)
            self.second_dock_center = self.one_class_svm(self.second_dock_list)
            self.facing = np.median(self.facing_list)
            self.aiming = math.pi - self.facing
            self.symbols_find = True

        if self.symbols_find:
            self.find_docking_path()

    def find_docking_path(self):
        # path for 1st and 2nd dockings
        # find docking points
        if self.first_dock_center != []:
            self.first_entry_path = [[self.first_dock_center[0] + self.clearance[0] * math.cos(self.facing),
                                      self.first_dock_center[1] + self.clearance[0] * math.sin(self.facing)],
                                     [self.first_dock_center[0] + self.clearance[1] * math.cos(self.facing),
                                      self.first_dock_center[1] + self.clearance[1] * math.sin(self.facing)]]

        if self.second_dock_center != []:
            self.second_entry_path = [[self.second_dock_center[0] + self.clearance[0] * math.cos(self.facing),
                                      self.second_dock_center[1] + self.clearance[0] * math.sin(self.facing)],
                                     [self.second_dock_center[0] + self.clearance[1] * math.cos(self.facing),
                                      self.second_dock_center[1] + self.clearance[1] * math.sin(self.facing)]]

    def one_class_svm(self, data_list):
        """ return support vector and thus cluster center """
        data_list = np.array(data_list)
        self.ocsvm.fit(data_list)
        sv = self.ocsvm.support_vectors_
        # find the sv's centroid, assume only one cluster.
        return [np.mean(sv[:,0]), np.mean(sv[:,1])]
        # return (np.median(sv[:,0]), np.median(sv[:,1]))

    def update_hold_moveto(self, hold_mv):
        self.hold_mv = hold_mv

    def update_hold_loiter(self, hold_loiter):
        self.hold_loiter = hold_loiter

    def random_walk(self, style):
        """ create random walk points and avoid valid centers """
        delta_y = 3
        target = None
        if style == "before_line":  # this time no gate data is known
            # do a gaussian distribution with center be the boat's current position and
            # sigma to be 5
            target = [random.gauss(self.x0, 5), random.gauss(self.y0, 5)]
            # while not target:
            #     if np.min(self.map_dim[0]) < candidate_target[0] < np.max(self.map_dim[0]) and\
            #         np.min(self.map_dim[1]) < candidate_target[1] < np.max(self.map_dim[0]):
            #         # it is after the gateline
            #         target = candidate_target
            #     else:
            #         target = None
        elif style == "near_line":  # gate data partially known, need to go around the line
            x_range = range(np.min(self.map_dim[0]), np.max(self.map_dim[0]), 5)
            y_estimate = [self.roughline.predict(x) - delta_y * self.before_roughline_sign for x in x_range]
            choices_idx = range(len(x_range))
            candidate_target_idx = random.choice(choices_idx)
            while not target:
                if np.min(self.map_dim[1]) < y_estimate[candidate_target_idx] < np.max(self.map_dim[1]):
                    # it is after the gateline
                    target = [x_range(candidate_target_idx), y_estimate(candidate_target_idx)]
                else:
                    target = None

        elif style == "along_line":  # gate data known, need to go to the three listener point
            target = random.choice([self.rwl, self.wwl, self.gwl])
        elif style == "after_gate":
            # do a uniform distribution by grid search
            x_range = range(np.min(self.map_dim[0]), np.max(self.map_dim[0]), 5)
            y_range = range(np.min(self.map_dim[1]), np.max(self.map_dim[1]), 5)
            grid = list(itertools.product(x_range, y_range))
            # filter out those who is before the gate line
            while not target:
                candidate_target = random.choice(grid)
                if (self.gateline.predict([candidate_target[0]]) - candidate_target[1]) * self.before_line_sign < 0:
                    # it is after the gateline
                    target = candidate_target
                else:
                    target = None
        return target + [0]

    def shutdown(self):
        pass


if __name__ == "__main__":
    try:
        totem=Docking("docking_planner")
    except rospy.ROSInterruptException:
        pass
