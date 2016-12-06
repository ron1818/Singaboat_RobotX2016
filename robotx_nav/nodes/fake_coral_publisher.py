#! /usr/bin/env python

import rospy
import random
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

"""
type of marker:
ARROW = 0   --> triangle
CUBE=1      --> cruciform
SPHERE=2    --> circle
CYLINDER=3  --> totem
LINE_STRIP=4--> rectangle
id: color:
RED=0
GREEN=1
BLUE=2
BLACK=3
WHITE=4
YELLOW=5
ORANGE=6
"""

class CoralPublisher():
    x0, y0 = 0, 0
    x, y = [0, 0], [0, 0]
    map_dim = np.array([[0, 40], [0, 40]])
    radius = 2.5
    def __init__(self, quandrants):
        rospy.init_node('coral_pub', anonymous=False)
        r = rospy.Rate(1)
        pub = rospy.Publisher("coral", MarkerArray, queue_size=50)
        self.odom_received = False
        rospy.Subscriber("odometry/filtered/global", Odometry, self.odom_callback, queue_size=None)
        while not self.odom_received:
            r.sleep()

        self.spawn_coral(quandrants)

        markerArray = MarkerArray()
        count = 0
        MARKERS_MAX = 2
        while not rospy.is_shutdown():
            if(count > MARKERS_MAX):
                markerArray.markers.pop(0)
            if self.within_range(self.x[0], self.y[0]):
                markerArray.markers.append(self.create_marker(self.x[0], self.y[0], "triangle"))
                count += 1
            if self.within_range(self.x[1], self.y[1]):
                markerArray.markers.append(self.create_marker(self.x[1], self.y[1], "cruciform"))
                count += 1

            pub.publish(markerArray)
            r.sleep()

    def within_range(self, x, y):
        if np.sqrt((self.x0 - x) ** 2 + (self.y0 - y) ** 2) > self.radius:
            return False
        else:
            return True

    def spawn_coral(self, quandrants):
        count = 0
        for q in quandrants:
            if q == 1:
                self.x[count] = random.randrange(np.mean(self.map_dim[0]), np.max(self.map_dim[0]))
                self.y[count] = random.randrange(np.mean(self.map_dim[1]), np.max(self.map_dim[1]))
            elif q == 2:
                self.x[count] = random.randrange(np.min(self.map_dim[0]), np.mean(self.map_dim[0]))
                self.y[count] = random.randrange(np.mean(self.map_dim[1]), np.max(self.map_dim[1]))
            elif q == 3:
                self.x[count] = random.randrange(np.min(self.map_dim[0]), np.mean(self.map_dim[0]))
                self.y[count] = random.randrange(np.min(self.map_dim[1]), np.mean(self.map_dim[1]))
            elif q == 4:
                self.x[count] = random.randrange(np.mean(self.map_dim[0]), np.max(self.map_dim[0]))
                self.y[count] = random.randrange(np.min(self.map_dim[1]), np.mean(self.map_dim[1]))

            count += 1

        print self.x, self.y

    def create_marker(self, x, y, symbol):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        # create start red totem
        marker.header.stamp = rospy.Time.now()
        if symbol == "triangle":
            marker.type = 0
        elif symbol == "cruciform":
            marker.type = 1
        elif symbol == "circle":
            marker.type = 2
        marker.id = 0  # red
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0

        return marker

    def odom_callback(self, msg):
        """ call back to subscribe, get odometry data:
        pose and orientation of the current boat,
        suffix 0 is for origin """
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        self.odom_received = True




if __name__ == "__main__":
    try:
         CoralPublisher([1,3])
    except rospy.ROSInterruptException:
        pass

