#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
import numpy as np
from sklearn.cluster import KMeans, DBSCAN
# import tf
# from math import pi, cos, sin
# from move_base_util import MoveBaseUtil
# import time

class TotemCoordinate(object):
    """ find coordinate of totem for task1 """
    red_list, green_list = list(), list()
    coordinate = [float('Inf'), float('Inf')]

    def __init__(self, nodename="totem_coordinate"):
        rospy.init_node(nodename)
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)

        self.kmeans = KMeans(n_clusters=2)

        # Subscribe to marker array publisher
        rospy.Subscriber("fake_marker_array", MarkerArray, self.markerarray_callback, queue_size=10)

        while not rospy.is_shutdown():
            red_centers, green_centers = list(), list()  # np.array([[0,0]]*2), np.array([[0,0]]*2)
            # k means clustering for both redlist and greenlist
            if len(self.red_list) > 0 and len(self.green_list) > 0:
                red_centers = self.kmeans_clustering(self.red_list)
            if len(self.green_list) > 0:
                green_centers = self.kmeans_clustering(self.green_list)

                # get the center points between each centers
                center_points =  np.array([[0.0,0.0]]*4)
                counter = 0
                for i in range(len(red_centers)):
                    for j in range(len(green_centers)):
                        center_points[counter] = (red_centers[i] + green_centers[j]) / 2.0
                        counter += 1
                print center_points

            r.sleep()


    def markerarray_callback(self, msg):
        """ calculate average over accumulate """
        for i in range(len(msg.markers)):
            if msg.markers[i].id == 0: # red
                self.red_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
            elif msg.markers[i].id == 1: # red
                self.green_list.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])

    def kmeans_clustering(self, data_list):
        """ calculate clustering and centers for a totem_list """
        self.kmeans.fit(data_list)
        return self.kmeans.cluster_centers_



if __name__ == "__main__":
    try:
        totem=TotemCoordinate("totem-coordinate")
    except rospy.ROSInterruptException:
        pass



