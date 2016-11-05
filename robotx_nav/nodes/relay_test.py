#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Vector3

def talker():
    pub = rospy.Publisher('chatter', Vector3, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg=Vector3(0,0,3)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
