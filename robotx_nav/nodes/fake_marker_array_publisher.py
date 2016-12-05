#! /usr/bin/env python
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

import rospy
import random
from visualization_msgs.msg import Marker, MarkerArray

class MarkerArrayPublisher():
    def __init__(self):
        pub = rospy.Publisher("fake_marker_array", MarkerArray, queue_size=50)
        rospy.init_node('fake_markerarray_pub', anonymous=False)
        r = rospy.Rate(1)

        count = 1
        MARKERS_MAX = 4
        start_red_x, start_red_y = -5, 10
        start_green_x, start_green_y = 5, 10

        end_red_x, end_red_y = -5, 20
        end_green_x, end_green_y = 5, 20

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
            if count % 4 == 1: # start red
                marker.id = 0  # red
                marker.pose.position.x = start_red_x + self.random_noise()
                marker.pose.position.y = start_red_y + self.random_noise()
                # markerArray.markers[0] = marker
            elif count % 4 == 2: # start green
                marker.id = 1  # green
                marker.pose.position.x = start_green_x + self.random_noise()
                marker.pose.position.y = start_green_y + self.random_noise()
                # markerArray.markers[1] = marker
            elif count % 4 == 3: # end red
                marker.id = 0  # red
                marker.pose.position.x = end_red_x + self.random_noise()
                marker.pose.position.y = end_red_y + self.random_noise()
                # markerArray.markers[2] = marker
            else: # end green
                marker.id = 1  # green
                marker.pose.position.x = end_green_x + self.random_noise()
                marker.pose.position.y = end_green_y + self.random_noise()
                marker.pose.position.z = 0
                # markerArray.markers[3] = marker
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



if __name__ == "__main__":
    try:
         MarkerArrayPublisher()
    except rospy.ROSInterruptException:
        pass
