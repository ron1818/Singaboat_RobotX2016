#!/usr/bin/env python
# license removed for brevity
""" rotation behavior
created by Weiwei @2016-10-21
"""


import rospy
from geometry_msgs.msg import Twist, Vector3


def rotation(ang):

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('rotation', anonymous=True)
    rate = rospy.Rate(10)
    an_vel=0.2
    duration=ang/an_vel;
    msg=Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, an_vel))

    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        current_time=rospy.get_time()
        if (current_time - start_time) > duration:
            pub.publish(Twist(Vector3(0, 0.0, 0.0), Vector3(0.0, 0.0, -2*an_vel)))
            rospy.sleep(0.3)
            pub.publish(Twist())
            break
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
	theta=rospy.get_param("/rotation_behavior/theta")
        rotation(theta)
    except rospy.ROSInterruptException:
        pass
