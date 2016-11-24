#!/usr/bin/env python
# license removed for brevity
""" blind reverse by time,
created by Reinaldo @2016-10-20
"""


import rospy
from geometry_msgs.msg import Twist, Vector3


def reverse(duration):

    duration = rospy.get_param("~duration", duration)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('reverse', anonymous=True)
    rate = rospy.Rate(10)

    vel = 0.3
    msg = Twist(Vector3(-vel, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        if (current_time - start_time) > duration:
            pub.publish(Twist(Vector3(vel, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
            rospy.sleep(1)
            pub.publish(Twist())
            break
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        reverse(duration=10)
    except rospy.ROSInterruptException:
        pass
