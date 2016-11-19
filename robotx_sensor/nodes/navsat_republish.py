#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import NavSatFix

class Navsat_Republish():
    callback_msg = NavSatFix()
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'gps_link')
        self.talker()

    def talker(self):
        rospy.Subscriber("fix", NavSatFix, self.callback)
        pub = rospy.Publisher("navsat/fix", NavSatFix, queue_size=50)
        rospy.init_node('navsat_republisher', anonymous=True)
        r = rospy.Rate(10)
        msg = NavSatFix()

        while not rospy.is_shutdown():
            msg = self.callback_msg
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            pub.publish(msg)
            r.sleep()

    def callback(self, msg):
        self.callback_msg = msg


if __name__ == "__main__":
    try:
        Navsat_Republish()
    except rospy.ROSInterruptException:
        pass



