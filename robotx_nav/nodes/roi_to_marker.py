#!/usr/bin/env python
import rospy
from sensor_msgs.msg import RegionOfInterest
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy
from move_base_waypoint import MoveTo

class ROI2Marker(object):
    x_offset, y_offset, height, width = 0, 0, 0, 0
    x_offset_list = list()
    y_offset_list = list()
    height_list = list()
    width_list = list()
    polar = [0, 0, 0]

    def __init__(self, nodename):
        self.image_width = rospy.get_param('~image_width', 640)
        self.totem_width = rospy.get_param('~totem_width', 0.25)
        self.rov = rospy.get_param('~rov', 1.41)
        self.frame_id = rospy.get_param('~frame_id', "odom")
        self.accumulate = rospy.get_param('~accumulate', 10)  # 30 images
        self.is_pub_marker = False
        self.image_count = 0
        self.moveto = MoveTo(nodename)

        rospy.Subscriber("bow/left/roi", RegionOfInterest, self.roi_callback, queue_size=50)

        while not rospy.is_shutdown():
            if self.is_pub_marker:
                self.moveto.shutdown()
                rospy.loginfo("sc")
                self.moveto(nodename="roi_moveto", target=self.polar)
            else:
                pass
            rospy.spin()

    def roi_callback(self, msg):
        # calculate average over accumulate
        if self.image_count <= self.accumulate:
            self.x_offset_list.append(msg.x_offset)
            self.y_offset_list.append(msg.y_offset)
            self.height_list.append(msg.height)
            self.width_list.append(msg.width)
            self.is_pub_marker = False
            self.image_count += 1
        else:
            self.x_offset = numpy.median(self.x_offset_list)
            self.y_offset = numpy.median(self.y_offset_list)
            self.height = numpy.median(self.height_list)
            self.width = numpy.median(self.width_list)
            self.x_offset_list = list()
            self.y_offset_list = list()
            self.height_list = list()
            self.width_list = list()
            self.polar = self.calculate_position()
            self.is_pub_marker = True
            self.image_count = 0

    def calculate_position(self):
        """ use xy_offset and height width to calculate position
        with known totem size """
        image_x_position = self.x_offset + self.width / 2.0
        x0, y0, yaw0 = 0, 0, 1.57
        # in polar form: theta: heading, r: distance
        theta = image_x_position / self.image_width * self.rov - self.rov / 2.0
        # act length = radius * arc radian
        r = self.totem_width / (self.width / self.image_width * self.rov)
        return [r, theta, 0]


if __name__ == "__main__":
    try:
        ROI2Marker("roi_arker")
    except rospy.ROSInterruptException:
        pass



