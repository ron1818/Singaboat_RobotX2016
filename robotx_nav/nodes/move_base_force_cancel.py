#! /usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalID

repetition = rospy.get_param("~repetition", 10)
rospy.init_node("cancel_goal", anonymous=True)

pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=10)
r = rospy.Rate(1)
counter = 0
while not rospy.is_shutdown() and (counter < repetition):
    msg = GoalID()
    # msg.stamp = rospy.Time()
    # msg.id = ""
    # print msg
    pub.publish(msg)
    r.sleep()
    counter += 1
