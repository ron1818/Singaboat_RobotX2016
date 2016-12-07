#! /usr/bin/env python

import rospy
import random
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

class PingerPublisher():
    x0, y0 = 0, 0
    map_dim = [[0, 40], [0, 40]]
    def __init__(self):
        rospy.init_node('pinger_pub', anonymous=False)
        r = rospy.Rate(1)
        gate_pub = rospy.Publisher("gate_totem", MarkerArray, queue_size=10)
        pinger_pub = rospy.Publisher("pinger", String, queue_size=10)
        self.odom_received = False
        rospy.Subscriber("odometry/filtered/global", Odometry, self.odom_callback, queue_size=None)
        while not self.odom_received:
            r.sleep()

        count = 1
        MARKERS_MAX = 12
        self.red = [8, 30]
        self.green = [8+7.07*3, 30-7.07*3]
        self.white1 = [8+7.07, 30-7.07]
        self.white2 = [8+7.07*2, 30-7.07*2]
        self.black = [35, 35]
        self.pinger = [8+7.07/2, 30-7.07/2]
        markerArray = MarkerArray()
        pinger = String()

        while not rospy.is_shutdown():
            if self.distance(self.red_totem) < threshold:
                if self.likely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("red"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("white"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("white1"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("white2"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("green"))
            if self.distance(self.green_totem) < threshold:
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("red"))
                if self.likely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("green"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("white1"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("white2"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("black"))
            if self.distance(self.white1_totem) < threshold:
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("red"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("green"))
                if self.likely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("white1"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("white2"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("black"))
            if self.distance(self.white2_totem) < threshold:
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("red"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("green"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("white1"))
                if self.likely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("white2"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("black"))
            if self.distance(self.black_totem) < threshold:
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("red"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("green"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("white1"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("white2"))
                if self.likely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("black"))

            if self.distance(self.pinger) < threshold * 0.75:
                pinger.data = "matching"
            else:
                pinger.data = "unknown"

            # Publish the MarkerArray
            pub.publish(markerArray)

            r.sleep()

    def create_marker(self, color):
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
        marker.type = marker.CYLINDER  # totem
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 0
        if color == "red":
            marker.id = 0  # red
            marker.pose.position.x = self.red[0] + self.random_noise()
            marker.pose.position.y = self.red[1] + self.random_noise()
        elif color == "green":
            marker.id = 1  # green
            marker.pose.position.x = self.green[0] + self.random_noise()
            marker.pose.position.y = self.green[1] + self.random_noise()
        elif color == "white1":
            marker.id = 4  # white 1
            marker.pose.position.x = self.white1[0] + self.random_noise()
            marker.pose.position.y = self.white1[1] + self.random_noise()
        elif color == "white2":
            marker.id = 4  # white 2
            marker.pose.position.x = self.white2[0] + self.random_noise()
            marker.pose.position.y = self.white2[1] + self.random_noise()
        elif color == "black":
            marker.id = 3  # black
            marker.pose.position.x = self.black[0] + self.random_noise()
            marker.pose.position.y = self.black[1] + self.random_noise()

        return marker

    def random_noise(self):
        return random.random() * 4.0 - 2.0

    def likely_spawn(self):
        return random.choice([True, False, False, False])

    def unlikely_spawn(self):
        return random.choice([False] * 25 + [True])

    def odom_callback(self, msg):
        """ call back to subscribe, get odometry data:
        pose and orientation of the current boat,
        suffix 0 is for origin """
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        self.odom_received = True




if __name__ == "__main__":
    try:
         PingerPublisher()
    except rospy.ROSInterruptException:
        pass

