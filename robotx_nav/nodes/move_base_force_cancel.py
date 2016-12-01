#! /usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalID

class ForceCancel(object):
    def __init__(self, nodename="cancel_goal", repetition=10):
        self.repetition = rospy.get_param("~repetition", repetition)
        rospy.init_node(nodename, anonymous=True)
        rospy.on_shutdown(self.shutdown)

        pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=10)
        r = rospy.Rate(1)
        counter = 0
        while not rospy.is_shutdown() and (counter < self.repetition):
            msg = GoalID()
            pub.publish(msg)
            r.sleep()
            counter += 1

    def shutdown():
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)
        pass
