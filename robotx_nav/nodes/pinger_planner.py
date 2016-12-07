#!/usr/bin/env python
import random
import itertools
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
import numpy as np
import math
from sklearn.cluster import KMeans, DBSCAN
from sklearn import svm
from sklearn.linear_model import LinearRegression


class Pinger(object):
    """ find coordinate of totem for task1 """
    red_list, green_list, white_list, black_list = list(), list(), list(), list()
    MAX_LENS = 20 # actually 21
    map_dim = [[0, 40], [0, 40]]
    x0, y0, yaw0 = None, None, None

    def __init__(self, nodename="pinger_planner"):
        rospy.init_node(nodename)
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param("~rate", 1)

        # self.kmeans = KMeans(n_clusters=2)
        self.ocsvm = svm.OneClassSVM(nu=0.1, kernel="rbf", gamma=0.1)

        # Subscribe to marker array publisher
        rospy.Subscriber("gate_totem", MarkerArray, self.markerarray_callback, queue_size=10)
        rospy.Subscriber("pinger", String, self.pinger_callback, queue_size=10)

        self.entry_gate = list()  # later need to fill in
        self.exit_gate = list()  # later need to fill in
        self.hold_loiter = False
        self.hold_mv = False
        while not x0:
            self.initial_position = [self.x0, self.y0]

    def planner(self):
        """ find the totems except black, line all of them, calculate the center, then find the gate entrance point,
        use the point to station keep and if it has pinger, enter and find the black totem,
        do a loiter on the totem and go back from the pinger's gate """
        self.loiter_target = list()
        self.mv_target = list()

        if self.gate_find:  # found the gate
            if self.entered:  # already entered
                if self.loiter_finished:  # already finish a loiter on black
                    self.mv_target = self.exit_gate
                else:  # need to loiter on the black totem
                    if self.black_center and not self.hold_loiter: # has a black center
                        self.loiter_target = [self.black_center, 6, 3, True]  # poly, r, ccw
                        self.hold_loiter = True
                    elif not self.black_center and not self.hold_mv:  # haven't find black center
                        self.mv_target = self.random_walk("after_gate")
                        self.hold_mv = True
            else:  # need to enter the gate
                if not self.hold_mv:
                    self.mv_target = self.entry_gate
                    self.hold_mv = True

        else:  # havent find the gate
            if self.gate_line is not None:  # find the gate line
                if not self.red_list or not self.green_list or not any(self.white_list):  # but still missing gate info
                    self.mv_target = self.random_walk("along_line")
                    self.hold_mv = True
            else:  # need to find the gate line
                self.mv_target = self.random_walk("before_gate")
                self.hold_mv = True

        return self.gate_find, self.loiter_target, self.mv_target

    def markerarray_callback(self, msg):
        """ calculate average over accumulate """
        # accumulate data
        for i in range(len(msg.markers)):
            if msg.markers[i].id == 0: # red
                if len(self.red_list) > self.MAX_LENS:
                    self.red_list.pop(0)
                self.red_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
            elif msg.markers[i].id == 1: # green
                if len(self.green_list) > self.MAX_LENS:
                    self.green_list.pop(0)
                self.green_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
            elif msg.markers[i].id == 4: # white
                if len(self.white_list) > 2 * self.MAX_LENS:
                    self.blue_list.pop(0)
                self.white_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
            elif msg.markers[i].id == 3: # black
                if len(self.black_list) > self.MAX_LENS:
                    self.black_list.pop(0)
                self.black_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])

    def find_gateline(self):
        # define regions for before, after and along the line

        # find the center
        if len(self.red_list) >= self.MAX_LENS:
            self.red_center = self.one_class_svm(self.red_list)
        if len(self.green_list) >= self.MAX_LENS:
            self.green_center = self.one_class_svm(self.green_list)
        if len(self.black_list) >= self.MAX_LENS:
            self.black_center = self.one_class_svm(self.black_list)
        if len(self.white_list) >= 2 * self.MAX_LENS:
            white_kmeans = KMeans(n_clusters=2)
            self.white_center = white_kmeans.fit(self.white_list)

        # find gate entrance
        if self.red_center and all(self.white_center) and self.green_center:  # all gate detected
            coordinates = np.concatenate(self.red_center, self.white_center, self.green_center)
            coord_x, coord_y = coordinates[:,0], coordinates[:,1]
            self.gateline = LinearRegression()
            # least square to get the gate line
            self.gateline.fit(coord_x.reshape((coord_x.shape[0],1)), coord_y)
            # k = self.gateline.coef_, b = self.gateline.intercept_
            # find the gate entry points
            if self.gateline.predict([self.initial_position[0]]) - self.initial_position[1] > 0:
                self.before_line_sign = 1  # 1 means > 0
            else:
                # "<" is the area where after line
                self.before_line_sign = -1  # -1 means < 0

            self.rwc, self.wwc, self.gwc = self.find_gate_entry()
            # find pinger listening point and mirrored point
            # listen point is for detecting pinger, and mirrored point is to supply the direction
            self.rwl, self.wwl, self.gwl, self.rwlm, self.wwlm, self.gwlm = self.find_listen_point()

    def find_gate_entry(self):
        # find red and white separations
        if self.distance(self.red_center, self.white_center[0]) < self.distance(self.red_center, self.white_center[1]):
            rwc = [np.mean(self.white_center[0,0], self.red_center[0]), np.mean(self.white_center[0,1], self.red_center[1])]
        else:
            rwc = [np.mean(self.white_center[1,0], self.red_center[0]), np.mean(self.white_center[1,1], self.red_center[1])]
        if self.separation(self.green_center, self.white_center[0]) < self.separation(self.green_center, self.white_center[1]):
            gwc = [np.mean(self.white_center[0,0], self.green_center[0]), np.mean(self.white_center[0,1], self.green_center[1])]
        else:
            gwc = [np.mean(self.white_center[1,0], self.green_center[0]), np.mean(self.white_center[1,1], self.green_center[1])]

        wwc = [np.mean(self.white_center[1,0], self.white_center[0,0]), np.mean(self.white_center[1,1], self.white_center[0,1])]

        return rwc, wwc, gwc

    def find_listen_point(self):
        # first is to find the two point that is with d to the gate entry
        theta = -1.0 / self.gateline.coef_
        rw = [self.rwc + self.gate_distance * math.cos(theta), self.rwc - self.gate_distance * math.cos(theta)]
        gw = [self.gwc + self.gate_distance * math.cos(theta), self.gwc - self.gate_distance * math.cos(theta)]
        ww = [self.wwc + self.gate_distance * math.cos(theta), self.wwc - self.gate_distance * math.cos(theta)]

        # l is listening point, lm is the mirrored point
        if self.distance(rw[0], [self.x0, self.y0]) < self.distance(rw[1], [self.x0, self.y0]):
            rwl = rw[0]
            rwlm = rw[1]
        if self.distance(gw[0], [self.x0, self.y0]) < self.distance(gw[1], [self.x0, self.y0]):
            gwl = gw[0]
            gwlm = gw[1]
        if self.distance(ww[0], [self.x0, self.y0]) < self.distance(ww[1], [self.x0, self.y0]):
            wwl = ww[0]
            wwlm = ww[1]

        return rwl, wwl, gwl, rwlm, wwlm, gwlm

    def distance(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def one_class_svm(self, data_list):
        """ return support vector and thus cluster center """
        data_list = np.array(data_list)
        self.ocsvm.fit(data_list)
        sv = self.ocsvm.support_vectors_
        # find the sv's centroid, assume only one cluster.
        return (np.mean(sv[:,0]), np.mean(sv[:,1]), 0)
        # return (np.median(sv[:,0]), np.median(sv[:,1]))

    def update_hold_moveto(self, hold_mv):
        self.hold_mv = hold_mv

    def update_hold_loiter(self, hold_loiter):
        self.hold_loiter = hold_loiter

    def random_walk(self, style):
        """ create random walk points and avoid valid centers """
        target = None
        if style == "before_gate":  # this time no gate data is known
            # do a gaussian distribution with center be the boat's current position and
            # sigma to be 5
            target = [random.gauss(self.x0, 5), random.gauss(self.y0, 5)]
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
        return target

    def shutdown(self):
        pass


if __name__ == "__main__":
    try:
        totem=Pinger("color_totem_planner")
    except rospy.ROSInterruptException:
        pass
