#!/usr/bin/env python
# license removed for brevity
""" rotation behavior
created by Weiwei @2016-10-21
"""


import rospy
import math
from geometry_msgs.msg import Twist, Point


def rotation(ang):

    ang = rospy.get_param("~theta", ang)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('rotation', anonymous=True)
    rate = rospy.Rate(10)
    an_vel = 1
    duration=abs(ang) / an_vel
    sign = math.copysign(1, ang)
    msg=Twist(Point(0.0, 0.0, 0.0), Point(0.0, 0.0, sign*an_vel))

    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        if (current_time - start_time) > duration:
            pub.publish(Twist(Point(0, 0.0, 0.0), Point(0.0, 0.0, -2*sign*an_vel)))
            rospy.sleep(1)
            pub.publish(Twist())
            break
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        rotation(ang=5)
    except rospy.ROSInterruptException:
        pass
