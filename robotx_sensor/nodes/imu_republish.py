#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu

class Imu_Republish():
    callback_msg = Imu()
    def __init__(self):
        self.talker()

    def talker(self):
        rospy.Subscriber("imu_in",Imu, self.callback)
        pub = rospy.Publisher("imu_out", Imu, queue_size=50)
        rospy.init_node('imu_republisher', anonymous=True)
        r = rospy.Rate(10)
        msg = Imu()

        while not rospy.is_shutdown():
            msg = self.callback_msg
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            r.sleep()

    def callback(self, msg):
        self.callback_msg = msg


if __name__ == "__main__":
    try:
        Imu_Republish()
    except rospy.ROSInterruptException:
        pass



