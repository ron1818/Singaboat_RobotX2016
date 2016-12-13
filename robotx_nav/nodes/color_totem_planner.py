#!/usr/bin/env python
import random
import itertools
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
import numpy as np
from sklearn.cluster import KMeans, DBSCAN
from sklearn import svm
import planner_utils

class ColorTotemPlanner(object):
    """ find coordinate of totem for task1 """
    isready=False
    red_list, green_list, yellow_list, blue_list = list(), list(), list(), list()
    red_center, green_center, yellow_center, blue_center = list(), list(), list(), list()
    MAX_LENS = 20 # actually 21

    def __init__(self, nodename="color_totem_coordinate", assigned=np.array([[2,False],[0,True],[1,False]])):
        """ assigned is a np.array:
        np.array[[color_id, is_ccw]], e.g.
        assigned = np.array([[2, False], [0, True], [1, False]])
        """
        rospy.init_node(nodename)
        rospy.on_shutdown(self.shutdown)

        self.id_counter = 0
        self.threshold = 5
        self.map_corners = np.array([[0,0], [0,40], [40,40], [40,0]])
        self.rate = rospy.get_param("~rate", 1)

        # self.kmeans = KMeans(n_clusters=2)
        self.ocsvm = svm.OneClassSVM(nu=0.1, kernel="rbf", gamma=0.1)

        # Subscribe to marker array publisher
        rospy.Subscriber("color_totem", MarkerArray, self.markerarray_callback, queue_size=10)

        self.allvisited = False # is all totems visited?
        self.assigned = assigned # n * 2 array
        self.visited = np.array([False] * len(self.assigned)) # n*1 array
        self.totem_find = np.array([False] * len(self.assigned)) # n*1 array
        self.center = np.array([[0, 0, 0]] * len(self.assigned)) # initialize center, n*3 array
        self.radius = np.array([2.5] * len(self.assigned)) # initialize center, n*3 array
        # self.visited_dict = {"red": False, "green": False, "blue": False, "yellow": False}
        # self.requested_dict =  {"red": False, "green": False, "blue": False, "yellow": False}
        self.hold_moveto = False # by default, let moveto to work
        self.hold_loiter = False # by default, let moveto to work
        # self.requested_moveto = False # by default, moveto is not requested
        # self.isready = False # is loiter ready? by default not go for loiter, only when loiter target identified

        self.exit_target = [-30, 15, 0]


    def planner(self):
        r = rospy.Rate(self.rate)
        # while not rospy.is_shutdown():
        self.loiter_target = list()
        self.moveto_target = list()

        # if not self.totem_find:
        #     self.moveto_target = planner_utils.random_walk(self.map_corners, style="unif", kwargs={"center": self.center})  # need to find centers
        # else:
        for i in range(len(self.assigned)):  # visit the totems in range
            # print "totem", i
            if not self.totem_find[i]:  # did not find the totem
                # print "totem", i, "not find"
                if not self.hold_moveto:
                    print "can move"
                    kwargs = {"center": self.center, "threshold": self.threshold}
                    self.moveto_target = planner_utils.random_walk(self.map_corners, style="unif", **kwargs)  # need to find centers
                    self.hold_moveto = True
            else:  # find the totem
                # print "find totem", i
                # print self.visited
                print "hold loiter?", self.hold_loiter
                print "visited", i, self.visited[i], self.visited[0:i], all(self.visited[0:i])
                if not self.visited[i] and all(self.visited[0:i]):  # previously all visited:
                    print "can loiter?"
                    if not self.hold_loiter:
                        print "can loiter"
                        # which totem, center, radius, ccw
                        self.loiter_target = [i, self.center[i], self.radius[i], self.assigned[i,1]]
                        self.hold_loiter = True
                        self.hold_moveto = True
                # else:
                #     if not self.hold_moveto:
                #         kwargs = {"center": self.center, "threshold": self.threshold}
                #         self.moveto_target = planner_utils.random_walk(self.map_corners, style="unif", **kwargs)  # need to find centers
                #         self.hold_moveto = True

        if all(self.visited):
            self.allvisited = True

        return self.totem_find, self.loiter_target, self.moveto_target, self.allvisited

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

        self.find_totem_center()

    def find_totem_center(self):
        # k means clustering for all color
        if len(self.red_list) >= self.MAX_LENS:  # have red totem info
            self.red_center = self.one_class_svm(self.red_list)
        if len(self.green_list) >= self.MAX_LENS:  # have green totem info
            self.green_center = self.one_class_svm(self.green_list)
        if len(self.blue_list) >= self.MAX_LENS:  # have blue totem info
            self.blue_center = self.one_class_svm(self.blue_list)
        if len(self.yellow_list) >= self.MAX_LENS:  # have yellow totem info
            self.yellow_center = self.one_class_svm(self.yellow_list)

        for i in range(len(self.assigned)):  # for each color
            if self.assigned[i,0] == 0 and self.red_center != []:  # red
                self.center[i,:] = self.red_center
                self.totem_find[i] = True
            elif self.assigned[i,0] == 1 and self.green_center != []:  # green
                self.center[i,:] = self.green_center
                self.totem_find[i] = True
            elif self.assigned[i,0] == 2 and self.blue_center != []:  # blue
                self.center[i,:] = self.blue_center
                self.totem_find[i] = True
            elif self.assigned[i,0] == 5 and self.yellow_center != []:  # yellow
                self.center[i,:] = self.yellow_center
                self.totem_find[i] = True

    def one_class_svm(self, data_list):
        """ return support vector and thus cluster center """
        data_list = np.array(data_list)
        self.ocsvm.fit(data_list)
        sv = self.ocsvm.support_vectors_
        # find the sv's centroid, assume only one cluster.
        return (np.mean(sv[:,0]), np.mean(sv[:,1]), 0)
        # return (np.median(sv[:,0]), np.median(sv[:,1]))

    def update_loiter(self, hold_loiter):
        """ update from external process"""
        # hold_loiter, visit_id = res
        self.hold_loiter = hold_loiter
        self.hold_moveto = hold_loiter
        # print "visit id", visit_id
        print "hold_loiter???? ????", self.hold_loiter
        self.visited[self.id_counter] = True
        print self.visited
        self.id_counter += 1

    def update_hold_moveto(self, hold_moveto):
        """ update from external process"""
        self.hold_moveto = hold_moveto

    def random_walk(self, centers): # use planner_utils.random_walk instead
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
