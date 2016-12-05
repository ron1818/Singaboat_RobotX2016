#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

class ForceCancel(object):
    def __init__(self, nodename="force_cancel", is_newnode=True, repetition=10):
        self.repetition = rospy.get_param("~repetition", repetition)
        if is_newnode:
            rospy.init_node(name=nodename, anonymous=False)
            rospy.on_shutdown(self.shutdown)

        pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)
        sub = rospy.Subscriber("move_base/goal", MoveBaseActionGoal, self.callback, queue_size=1)
        rospy.wait_for_message("move_base/goal", MoveBaseActionGoal, 60)
        r = rospy.Rate(1)
        counter = 0
        while not rospy.is_shutdown() and (counter < self.repetition):
            msg = GoalID()
            msg.id = self.id
            pub.publish(msg)
            r.sleep()
            counter += 1
    def callback(self, msg):
        self.id = msg.goal_id.id


    def shutdown(self):
        rospy.loginfo("cancel job finished")
        rospy.sleep(1)
        pass

if __name__ == "__main__":
    fc = ForceCancel('force_cancel', False, 5)
