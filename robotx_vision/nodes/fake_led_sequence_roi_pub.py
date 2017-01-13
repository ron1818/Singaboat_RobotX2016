#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image, RegionOfInterest

rospy.init_node("fake_led_seq")
pub = rospy.Publisher("/fixed_cam/led_sequence_roi", RegionOfInterest, queue_size =5)
while not rospy.is_shutdown():
    msg = RegionOfInterest()
    msg.x_offset = 300
    msg.y_offset = 200
    msg.width = 60
    msg.height = 60
    pub.publish(msg)
    rospy.sleep(1)

