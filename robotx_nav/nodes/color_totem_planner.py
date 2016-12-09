#!/usr/bin/env python
import random
import itertools
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
import numpy as np
from sklearn.cluster import KMeans, DBSCAN
from sklearn import svm
class ColorTotemPlanner(object):
    """ find coordinate of totem for task1 """
    isready=False
    red_list, green_list, yellow_list, blue_list = list(), list(), list(), list()
    MAX_LENS = 20 # actually 21

    def __init__(self, nodename="color_totem_coordinate"):
        rospy.init_node(nodename)
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param("~rate", 1)

        # self.kmeans = KMeans(n_clusters=2)
        self.ocsvm = svm.OneClassSVM(nu=0.1, kernel="rbf", gamma=0.1)

        # Subscribe to marker array publisher
        rospy.Subscriber("color_totem", MarkerArray, self.markerarray_callback, queue_size=10)

        self.visited_dict = {"red": False, "green": False, "blue": False, "yellow": False}
        self.requested_dict =  {"red": False, "green": False, "blue": False, "yellow": False}
        self.hold_moveto = False # by default, let moveto to work
        self.requested_moveto = False # by default, moveto is not requested
        self.isready = False # is loiter ready? by default not go for loiter, only when loiter target identified

        self.exit_target = [-30, 15, 0]

    def planner(self):
        r = rospy.Rate(self.rate)
        # while not rospy.is_shutdown():
        self.allvisited = False # is all totems visited?
        self.loiter_target = list()
        self.moveto_target = list()
        red_centers, green_centers, blue_centers, yellow_centers =\
               list(), list(), list(), list()  # np.array([[0,0]]*2), np.array([[0,0]]*2)
        # # k means clustering for both redlist and greenlist
        if len(self.red_list) > self.MAX_LENS:  # have red totem info
            red_centers = self.one_class_svm(self.red_list)
        if len(self.green_list) > self.MAX_LENS:  # have green totem info
            green_centers = self.one_class_svm(self.green_list)
        if len(self.blue_list) > self.MAX_LENS:  # have blue totem info
            blue_centers = self.one_class_svm(self.blue_list)
        if len(self.yellow_list) > self.MAX_LENS:  # have yellow totem info
            yellow_centers = self.one_class_svm(self.yellow_list)

            # print "r", red_centers, "g", green_centers, "b", blue_centers, "y", yellow_centers

        # sequence: green blue yellow red
        if green_centers != [] and not self.visited_dict["green"] and not self.requested_dict["green"]:
            # loiter counterclockwise
            print "start green"
            self.isready = True  # go for loiter
            self.hold_moveto = True # not go for moveto
            self.loiter_target = ["green", green_centers, 2.5, 6, True]
            self.requested_dict["green"] = True # request to go for loiter, on hold for loiter

        if blue_centers != [] and not self.visited_dict["blue"] and self.visited_dict["green"] and not self.requested_dict["blue"]:
            # loiter clockwise
            print "start blue"
            self.isready = True
            self.hold_moveto = True
            self.loiter_target = ["blue", blue_centers, 2.5, 6, False]
            self.requested_dict["blue"] = True
        if red_centers != [] and not self.visited_dict["red"] and self.visited_dict["blue"] and self.visited_dict["green"] and not self.requested_dict["red"]:
            # loiter clockwise
            print "start red"
            self.isready = True
            self.hold_moveto = True
            self.loiter_target = ["red", red_centers, 2.5, 6, False]
            self.requested_dict["red"] = True
        if yellow_centers != [] and not self.visited_dict["yellow"] and self.visited_dict["red"] and self.visited_dict["blue"] and self.visited_dict["green"] and not self.requested_dict["yellow"]:
            # loiter counterclockwise
            print "start yellow"
            self.isready = True
            self.hold_moveto = True
            self.loiter_target = ["yellow", yellow_centers, 2.5, 6, True]
            self.requested_dict["yellow"] = True

        if all(self.visited_dict.values()):  # all visited
            # exit the region
            self.allvisited = True
            self.moveto_target = self.exit_target
            print "all visited"

        # moveto not hold, not ready for loiter, still did not finish, go for moveto
        if not self.isready and not all(self.visited_dict.values()):
            if not self.hold_moveto:
                self.moveto_target = list(self.random_walk([red_centers, green_centers, blue_centers, yellow_centers])) + [0]
                self.hold_moveto = True  # executing moveto, hold ongoing moveto task
                self.requested_moveto = False
                # hold_moveto = False  # executing moveto, hold the next
            else:  # moveto not hold, not ready for loiter, still did not finish, go for moveto
                self.requested_moveto = True

        return self.isready, self.loiter_target, self.moveto_target, self.allvisited, self.requested_moveto

        # if cannot make all visited, random walk move to
        #   r.sleep()

    def markerarray_callback(self, msg):
        """ calculate average over accumulate """
        for i in range(len(msg.markers)):
            if msg.markers[i].id == 0: # red
                if len(self.red_list) > self.MAX_LENS:
                    self.red_list.pop(0)
                self.red_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
            elif msg.markers[i].id == 1: # green
                if len(self.green_list) > self.MAX_LENS:
                    self.green_list.pop(0)
                self.green_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
            elif msg.markers[i].id == 2: # blue
                if len(self.blue_list) > self.MAX_LENS:
                    self.blue_list.pop(0)
                self.blue_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
            elif msg.markers[i].id == 5: # yellow
                if len(self.yellow_list) > self.MAX_LENS:
                    self.yellow_list.pop(0)
                self.yellow_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])

    def one_class_svm(self, data_list):
        """ return support vector and thus cluster center """
        data_list = np.array(data_list)
        self.ocsvm.fit(data_list)
        sv = self.ocsvm.support_vectors_
        # find the sv's centroid, assume only one cluster.
        return (np.mean(sv[:,0]), np.mean(sv[:,1]), 0)
        # return (np.median(sv[:,0]), np.median(sv[:,1]))

    def update_visit(self, visited_dict):
        """ update from external process"""
        self.visited_dict = visited_dict
        self.isready = False  # loiter finish, make it is not ready

    def update_hold_moveto(self, hold_moveto):
        """ update from external process"""
        self.hold_moveto = hold_moveto

    def random_walk(self, centers):
        """ create random walk points and avoid valid centers """
        self.mapsize = (-30, 30)
        x_range = range(self.mapsize[0], 0, 5)
        y_range = range(0, self.mapsize[1], 5)
        grid = list(itertools.product(x_range, y_range))
        return random.choice(grid)

    def shutdown(self):
        pass





if __name__ == "__main__":
    try:
        totem=ColorTotemPlanner("color_totem_planner")
    except rospy.ROSInterruptException:
        pass
