#! /usr/bin/env python

import rospy
import random
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

class MarkerArrayPublisher():
    x0, y0 = 0, 0
    def __init__(self):
        rospy.init_node('color_totem_pub', anonymous=False)
        r = rospy.Rate(1)
        pub = rospy.Publisher("color_totem", MarkerArray, queue_size=50)
        self.odom_received = False
        rospy.Subscriber("odometry/filtered/global", Odometry, self.odom_callback, queue_size=None)
        while not self.odom_received:
            r.sleep()

        # map area (0,0) to (-30,30), (-15, 15) is a division point for observation
        count = 1
        MARKERS_MAX = 12
        self.red_x, self.red_y = -10, 5
        self.green_x, self.green_y = -18, 8
        self.yellow_x, self.yellow_y = -16, 17
        self.blue_x, self.blue_y = -14, 20
        markerArray = MarkerArray()

        while not rospy.is_shutdown():
            if self.x0 > -15 and self.y0 < 15:  # more chance to see red
                if self.likely_spawn():
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
                    markerArray.markers.append(self.create_marker("blue"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("yellow"))
            elif self.x0 < -15 and self.y0 < 15:  # more chance to see green
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
                    markerArray.markers.append(self.create_marker("blue"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("yellow"))
            elif self.x0 > -15 and self.y0 > 15:  # more chance to see blue
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
                    markerArray.markers.append(self.create_marker("blue"))
                if self.unlikely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("yellow"))
            elif self.x0 < -15 and self.y0 > 15:  # more chance to see yellow
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
                    markerArray.markers.append(self.create_marker("blue"))
                if self.likely_spawn():
                    count += 1
                    if(count > MARKERS_MAX):
                        markerArray.markers.pop(0)
                    markerArray.markers.append(self.create_marker("yellow"))


            # We add the new marker to the MarkerArray, removing the oldest
            # marker from it when necessary
            # if(count > MARKERS_MAX):
            #     markerArray.markers.pop(0)

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
            marker.pose.position.x = self.red_x + self.random_noise()
            marker.pose.position.y = self.red_y + self.random_noise()
        elif color == "green":
            marker.id = 1  # green
            marker.pose.position.x = self.green_x + self.random_noise()
            marker.pose.position.y = self.green_y + self.random_noise()
        elif color == "blue":
            marker.id = 5  # yellow
            marker.pose.position.x = self.yellow_x + self.random_noise()
            marker.pose.position.y = self.yellow_y + self.random_noise()
        elif color == "yellow":
            marker.id = 2  # blue
            marker.pose.position.x = self.blue_x + self.random_noise()
            marker.pose.position.y = self.blue_y + self.random_noise()

        return marker

    def random_noise(self):
        return random.random() * 6.0 - 3.0

    def likely_spawn(self):
        return random.choice([True, False, False, False])

    def unlikely_spawn(self):
        return random.choice([False] * 15 + [True])

    def odom_callback(self, msg):
        """ call back to subscribe, get odometry data:
        pose and orientation of the current boat,
        suffix 0 is for origin """
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        self.odom_received = True




if __name__ == "__main__":
    try:
         MarkerArrayPublisher()
    except rospy.ROSInterruptException:
        pass

