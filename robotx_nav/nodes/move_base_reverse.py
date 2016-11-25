#!/usr/bin/env python
# license removed for brevity
""" blind reverse by time,
created by Reinaldo @2016-10-20
"""


# import rospy
# from move_base_util import MoveBaseUtil
#
# if __name__ == "__main__":
#     is_timed = rospy.get_param("~is_timed", False)
#     distance = rospy.get_param("~distance", 5)
#     speed = rospy.get_param("~distance", -1)
#     duration = rospy.get_param("~duration", 5)
#     util = MoveBaseUtil("reverse_test")
#     if not is_timed:
#         util.reverse_tf(distance=distance, speed=speed)
#     else:
#         util.reverse_time(duration=duration, speed=speed)

import rospy
from geometry_msgs.msg import Twist, Vector3


def reverse(duration, speed):

    speed = rospy.get_param("~speed", speed)
    duration = rospy.get_param("~duration", duration)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('reverse', anonymous=True)
    rate = rospy.Rate(10)

    msg = Twist(Vector3(speed, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    rate.sleep()
    start_time = rospy.get_time()

    try:
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            if (current_time - start_time) > duration:
                pub.publish(Twist(Vector3(-speed, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
                rospy.sleep(1)
                pub.publish(Twist())
                break
            pub.publish(msg)
            rate.sleep()
    except:
        pub.publish(Twist())



if __name__ == '__main__':
    try:
        reverse(duration=10, speed=-0.3)
    except rospy.ROSInterruptException:
        pass
