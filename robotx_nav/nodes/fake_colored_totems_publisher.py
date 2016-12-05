#! /usr/bin/env python

import rospy
import random
from visualization_msgs.msg import Marker, MarkerArray

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
    def __init__(self):
        pub = rospy.Publisher("color_totem", MarkerArray, queue_size=50)
        rospy.init_node('color_totem_pub', anonymous=False)
        r = rospy.Rate(1)

        count = 1
        MARKERS_MAX = 4
        red_x, red_y = -10, 5
        green_x, green_y = 5, 10
        yellow_x, yellow_y = -2, 17
        blue_x, blue_y = 14, 20
        markerArray = MarkerArray()

        while not rospy.is_shutdown():
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
            if self.spawn():
                if count % 4 == 1: # red
                    marker.id = 0  # red
                    marker.pose.position.x = red_x + self.random_noise()
                    marker.pose.position.y = red_y + self.random_noise()
                    # markerArray.markers[0] = marker
                elif count % 4 == 2: # green
                    marker.id = 1  # green
                    marker.pose.position.x = green_x + self.random_noise()
                    marker.pose.position.y = green_y + self.random_noise()
                    # markerArray.markers[1] = marker
                elif count % 4 == 3: # yellow
                    marker.id = 5  # yellow
                    marker.pose.position.x = yellow_x + self.random_noise()
                    marker.pose.position.y = yellow_y + self.random_noise()
                    # markerArray.markers[2] = marker
                elif count % 4 == 0: # blue
                    marker.id = 2  # blue
                    marker.pose.position.x = blue_x + self.random_noise()
                    marker.pose.position.y = blue_y + self.random_noise()

            markerArray.markers.append(marker)

            # We add the new marker to the MarkerArray, removing the oldest
            # marker from it when necessary
            if(count > MARKERS_MAX):
                markerArray.markers.pop(0)

            # Publish the MarkerArray
            pub.publish(markerArray)
            count += 1

            r.sleep()

    def random_noise(self):
        return random.random() * 6.0 - 3.0

    def spawn(self):
        return random.choice([True, False])



if __name__ == "__main__":
    try:
         MarkerArrayPublisher()
    except rospy.ROSInterruptException:
        pass



