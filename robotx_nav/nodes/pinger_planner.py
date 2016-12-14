#!/usr/bin/env python
import random
import itertools
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Int8, Float64
import numpy as np
import math
from sklearn.cluster import KMeans, DBSCAN
from sklearn import svm
from sklearn.linear_model import LinearRegression
from move_base_loiter import Loiter
from move_base_waypoint import MoveTo
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class Pinger(object):
    """ find coordinate of totem for task1 """
    red_list, green_list, white_list, black_list = list(), list(), list(), list()
    MIN_LENS = 5 # actually 21
    MAX_LENS = 50 # actually 21
    map_dim = [[0, 40], [0, 40]]
    pinger_list = list()
    pinger_center = list()
    red_center, green_center, black_center, white_center = list(), list(), list(), list()
    x0, y0, yaw0 = 0, 0, 0
    initial_position = list()

    def __init__(self, nodename="pinger_planner"):
        rospy.init_node(nodename)
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param("~rate", 1)
        self.tf_listener = tf.TransformListener()

        self.kmeans = KMeans(n_clusters=2)
        self.ocsvm = svm.OneClassSVM(nu=0.1, kernel="rbf", gamma=0.1)
        self.moveto = MoveTo("moveto", is_newnode=False, target=None, mode=1, mode_param=1, is_relative=False)
        self.loiter = Loiter("loiter", is_newnode=False, target=None, is_relative=False)

        # Subscribe to marker array publisher
        self.odom_received = False
        rospy.Subscriber("odometry/filtered/global", Odometry, self.odom_callback, queue_size=None)
        while not self.odom_received:
            pass
        rospy.Subscriber("filtered_marker_array", MarkerArray, self.markerarray_callback, queue_size=10)
        # rospy.Subscriber("pinger", Float64, self.pinger_callback, queue_size=10)
        rospy.Subscriber("hydrophone", Int8, self.pinger_callback, queue_size=10)


        self.pinger_threshold = 65
        self.entry_distance = 2
        self.exited = False
        self.gate_totem_find = False
        self.pinger_find = False
        self.center_gate = list()  # later need to fill in
        self.entry_gate = list()  # later need to fill in
        self.exit_gate = list()  # later need to fill in
        self.hold_loiter = False
        self.hold_mv = False
        # while not self.x0:
        self.initial_position = [self.x0, self.y0]

        self.planner()

    def planner(self):
        """ find the totems except black, line all of them, calculate the center, then find the gate entrance point,
        use the point to station keep and if it has pinger, enter and find the black totem,
        do a loiter on the totem and go back from the pinger's gate """
        r = rospy.Rate(self.rate)
        self.loiter_target = list()
        self.mv_target = list()

        while not self.gate_totem_find:
            if not self.red_center or not self.green_center or len(self.white_center) < 2:
                print "a"
                self.moveto.respawn(self.random_walk("before_line"))
            else:
                print "b"
                self.moveto.respawn(self.random_walk("near_line"))
            r.sleep()
        else:  # find the gate totems
            # # get the gate line, done immediatedly after gate totem find
            print "gate totem find"
            while not self.pinger_find:
                print "go along line"
                self.moveto.respawn(self.random_walk("along_line"))
                r.sleep()
            else:
                print "pinger find"
                # find the entry point who has pinger
                self.locate_pinger_gate()
                print "enter gate"
                self.moveto.respawn(self.exit_gate)
                self.moveto.respawn(self.center_gate)
                print self.entry_gate
                self.moveto.respawn(self.entry_gate)
                while not self.black_totem_find:
                    # need to random walk after line
                    print "find black"
                    self.moveto.respawn(self.random_walk("after_line"))
                    r.sleep()
                else:  # find the black totem
                    # loiter the black totem
                    print "loiter"
                    print self.black_center
                    self.loiter.respawn(self.black_center)
                    # followed by exit the gate
                    print "exit"
                    self.moveto.respawn(self.entry_gate)
                    self.moveto.respawn(self.center_gate)
                    print self.exit_gate
                    self.moveto.respawn(self.exit_gate)
                    # exit complete
                    print "complete"

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

    def pinger_callback(self, msg):
        """ get pinger information """
        # if msg.data > self.pinger_threshold
        if msg.data == 1:
            self.pinger_list.append([self.x0, self.y0])

        self.find_pinger_center()

    def find_pinger_center(self):
        if len(self.pinger_list) >= self.MAX_LENS:
            self.pinger_center = self.one_class_svm(self.pinger_list)
            self.pinger_find = True
        else:
            self.pinger_find = False
            print "pinger not find"

    def markerarray_callback(self, msg):
        """ calculate average over accumulate """
        # accumulate data
        for i in range(len(msg.markers)):
            if msg.markers[i].id == 0:  # red
                if len(self.red_list) > self.MAX_LENS:
                    self.red_list.pop(0)
                self.red_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
            elif msg.markers[i].id == 1:  # green
                if len(self.green_list) > self.MAX_LENS:
                    self.green_list.pop(0)
                self.green_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
            elif msg.markers[i].id == 4:  # white
                if len(self.white_list) > 2 * self.MAX_LENS:
                    self.white_list.pop(0)
                self.white_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
            elif msg.markers[i].id == 3:  # black
                if len(self.black_list) > self.MAX_LENS:
                    self.black_list.pop(0)
                self.black_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])

            # print self.red_list, self.white_list, self.green_list

        self.find_gateline()

    def find_gateline(self):
        # define regions for before, after and along the line
        # find the center
        if len(self.red_list) >= self.MIN_LENS:
            self.red_center = self.one_class_svm(self.red_list)
        if len(self.green_list) >= self.MIN_LENS:
            self.green_center = self.one_class_svm(self.green_list)
        if len(self.black_list) >= self.MIN_LENS:
            self.black_center = self.one_class_svm(self.black_list)
            self.black_totem_find = True  # for planner
        if len(self.white_list) >= 2 * self.MIN_LENS:
            self.kmeans.fit(self.white_list)
            self.white_center = self.kmeans.cluster_centers_

        # print self.red_center, self.white_center, self.green_center


        # find gate entrance
        if self.red_center != [] and len(self.white_center) == 2 and self.green_center != []:  # all gate detected
            coordinates = np.concatenate(([self.red_center], self.white_center, [self.green_center]))
            coord_x, coord_y = coordinates[:,0], coordinates[:,1]
            self.gateline = LinearRegression()
            # least square to get the gate line
            self.gateline.fit(coord_x.reshape((coord_x.shape[0],1)), coord_y)
            # k = self.gateline.coef_, b = self.gateline.intercept_
            # find the gate entry points
            # find the before the line sign:
            # "1" is the area where before line
            # "-1" is the area where after line
            self.before_line_sign = np.sign(self.gateline.predict([self.initial_position[0]]) - self.initial_position[1])
            self.find_gate_entry()
            self.gate_totem_find = True

        elif self.red_list or self.green_list or self.white_list:  # use currently available data
            coordinates = [[self.x0, self.y0]]
            if self.red_list:
                coordinates = np.concatenate((coordinates, self.red_list))
            if self.white_list:
                coordinates = np.concatenate((coordinates, self.white_list))
            if self.green_list:
                coordinates = np.concatenate((coordinates, self.green_list))

            coord_x, coord_y = coordinates[:,0], coordinates[:,1]
            self.roughline = LinearRegression()
            # least square to get the gate line
            self.roughline.fit(coord_x.reshape((coord_x.shape[0],1)), coord_y)
            self.before_roughline_sign = np.sign(self.roughline.predict([self.initial_position[0]]) - self.initial_position[1])
        else: # no data
            pass

    def find_gate_entry(self):
        # this function will be called when gate_totem_find and pinger_find
        # find red and white separations
        if self.distance(self.red_center, self.white_center[0]) < self.distance(self.red_center, self.white_center[1]):
            self.rwc = [np.mean([self.white_center[0,0], self.red_center[0]]), np.mean([self.white_center[0,1], self.red_center[1]])]
        else:
            self.rwc = [np.mean([self.white_center[1,0], self.red_center[0]]), np.mean([self.white_center[1,1], self.red_center[1]])]
        if self.distance(self.green_center, self.white_center[0]) < self.distance(self.green_center, self.white_center[1]):
            self.gwc = [np.mean([self.white_center[0,0], self.green_center[0]]), np.mean([self.white_center[0,1], self.green_center[1]])]
        else:
            self.gwc = [np.mean([self.white_center[1,0], self.green_center[0]]), np.mean([self.white_center[1,1], self.green_center[1]])]

        self.wwc = [np.mean([self.white_center[1,0], self.white_center[0,0]]), np.mean([self.white_center[1,1], self.white_center[0,1]])]

        self.rwl, self.wwl, self.gwl, self.rwlm, self.wwlm, self.gwlm = self.find_listen_point()

    def locate_pinger_gate(self):
        # find the gate now
        # self.find_pinger_center()  # find pinger center
        # shortest distance to the identified gate center
        print "pinger center", self.pinger_center
        print "rwl", self.rwl
        pinger_to_entry_distance = [self.distance(self.pinger_center, self.rwl),
                                    self.distance(self.pinger_center, self.wwl),
                                    self.distance(self.pinger_center, self.gwl)]

        shortest_d = np.argmin(pinger_to_entry_distance)
        # entry_gate identified
        # must be inside
        # exit gate must be outside
        self.center_gate = [self.rwc, self.wwc, self.gwc][shortest_d] + [0]
        self.entry_gate = [self.rwlm, self.wwlm, self.gwlm][shortest_d] + [0]
        self.exit_gate = [self.rwl, self.wwl, self.gwl][shortest_d] + [0]

    def find_listen_point(self):
        # find pinger listening point and mirrored point
        # listen point is for detecting pinger, and mirrored point is to supply the direction
        # first is to find the two point that is with d to the gate entry
        theta = -1.0 / self.gateline.coef_
        rw = [[self.rwc[0] + self.entry_distance * math.cos(theta), self.rwc[1] + self.entry_distance * math.sin(theta)],
              [self.rwc[0] - self.entry_distance * math.cos(theta), self.rwc[1] - self.entry_distance * math.sin(theta)]]
        ww = [[self.wwc[0] + self.entry_distance * math.cos(theta), self.wwc[1] + self.entry_distance * math.sin(theta)],
              [self.wwc[0] - self.entry_distance * math.cos(theta), self.wwc[1] - self.entry_distance * math.sin(theta)]]
        gw = [[self.gwc[0] + self.entry_distance * math.cos(theta), self.gwc[1] + self.entry_distance * math.sin(theta)],
              [self.gwc[0] - self.entry_distance * math.cos(theta), self.gwc[1] - self.entry_distance * math.sin(theta)]]

        # l is listening point, lm is the mirrored point
        if self.before_line_sign * (self.gateline.predict([rw[0][0]]) - rw[0][1]) > 0:
            rwl = rw[0]
            rwlm = rw[1]
        else:
            rwl = rw[1]
            rwlm = rw[0]
        if self.before_line_sign * (self.gateline.predict([gw[0][0]]) - gw[0][1]) > 0:
            gwl = gw[0]
            gwlm = gw[1]
        else:
            gwl = gw[1]
            gwlm = gw[0]
        if self.before_line_sign * (self.gateline.predict([ww[0][0]]) - ww[0][1]) > 0:
            wwl = ww[0]
            wwlm = ww[1]
        else:
            wwl = ww[1]
            wwlm = ww[0]

        return rwl, wwl, gwl, rwlm, wwlm, gwlm

    def distance(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

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
        totem=Pinger("pinger_planner")
    except rospy.ROSInterruptException:
        pass
