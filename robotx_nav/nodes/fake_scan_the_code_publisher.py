#! /usr/bin/env python

import rospy
import random
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String

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

class ScanTheCodePublisher():
    x0, y0 = 0, 0
    x, y = 0, 0
    map_dim = np.array([[0, 40], [0, 40]])
    radius = 7.5
    def __init__(self):
        rospy.init_node('code_pub', anonymous=False)
        r = rospy.Rate(1)
        marker_pub = rospy.Publisher("totem_lamp", MarkerArray, queue_size=10)
        led_pub = rospy.Publisher("led_sequence", String, queue_size=10)
        self.odom_received = False
        rospy.Subscriber("odometry/filtered/global", Odometry, self.odom_callback, queue_size=None)
        while not self.odom_received:
            r.sleep()

        markerArray = MarkerArray()
        led_sequence = String()
        count = 0
        timer = 0
        MARKERS_MAX = 1

        while not rospy.is_shutdown():
            self.spawn_led()
            if count > MARKERS_MAX and len(markerArray.markers) > 0:
                markerArray.markers.pop(0)
            if self.within_range(self.x, self.y):
                markerArray.markers.append(self.create_marker())
                if timer > 100:
                    led_sequence.data = "found"
                else:
                    led_sequence.data = "unknown"
                    timer += 1
                count += 1

            marker_pub.publish(markerArray)
            led_pub.publish(led_sequence)
            r.sleep()

    def within_range(self, x, y):
        if np.sqrt((self.x0 - x) ** 2 + (self.y0 - y) ** 2) > self.radius:
            return False
        else:
            return True

    def spawn_led(self):
        self.x = random.gauss(np.mean(self.map_dim[0]), 0.01 * np.ptp(self.map_dim[0]))
        self.y = random.gauss(np.mean(self.map_dim[1]), 0.01 * np.ptp(self.map_dim[1]))
        # print self.x, self.y

    def create_marker(self):
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
        marker.type = 3
        marker.id = 0  # red
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
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
         ScanTheCodePublisher()
    except rospy.ROSInterruptException:
        pass

