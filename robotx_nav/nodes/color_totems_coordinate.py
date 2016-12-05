#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
import numpy as np
from sklearn.cluster import KMeans, DBSCAN
from sklearn import svm
# import tf
# from math import pi, cos, sin
# from move_base_util import MoveBaseUtil
# import time

class ColorTotemCoordinate(object):
    """ find coordinate of totem for task1 """
    red_list, green_list, yellow_list, blue_list = list(), list(), list(), list()
    MAX_LENS = 20

    def __init__(self, nodename="color_totem_coordinate"):
        rospy.init_node(nodename)
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param("~rate", 1)
        r = rospy.Rate(self.rate)

        self.kmeans = KMeans(n_clusters=2)
        self.ocsvm = svm.OneClassSVM(nu=0.1, kernel="rbf", gamma=0.1)

        # Subscribe to marker array publisher
        rospy.Subscriber("color_totem", MarkerArray, self.markerarray_callback, queue_size=10)

        while not rospy.is_shutdown():
            red_centers, green_centers, blue_centers, yellow_centers =\
                   list(), list(), list(), list()  # np.array([[0,0]]*2), np.array([[0,0]]*2)
            # # k means clustering for both redlist and greenlist
            # if len(self.red_list) > 0 and len(self.green_list) > 0 \
            #     and len(self.blue_list) > 0 and len(self.yellow_list) > 0:
            #     print len(self.red_list), len(self.green_list), len(self.blue_list), len(self.yellow_list)
            if len(self.red_list) > 0:  # have red totem info
                print len(self.red_list)
                red_centers = self.one_class_svm(self.red_list)
            if len(self.green_list) > 0:  # have green totem info
                print len(self.green_list)
                green_centers = self.one_class_svm(self.green_list)
            if len(self.blue_list) > 0:  # have blue totem info
                print len(self.blue_list)
                blue_centers = self.one_class_svm(self.blue_list)
            if len(self.yellow_list) > 0:  # have yellow totem info
                print len(self.yellow_list)
                yellow_centers = self.one_class_svm(self.yellow_list)

                print red_centers, green_centers, blue_centers, yellow_centers

            r.sleep()


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
        return (np.mean(sv[:,0]), np.mean(sv[:,1]))
        # return (np.median(sv[:,0]), np.median(sv[:,1]))

    def shutdown(self):
        pass



if __name__ == "__main__":
    try:
        totem=ColorTotemCoordinate("color-totem-coordinate")
    except rospy.ROSInterruptException:
        pass



