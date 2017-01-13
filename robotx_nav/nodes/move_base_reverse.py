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
from move_base_util import MoveBaseUtil


class Reversing(MoveBaseUtil):

    def __init__(self, nodename="reverse", is_newnode=True, mode="timed", speed=-2.3, duration=10, distance=5):
        MoveBaseUtil.__init__(self, nodename, is_newnode)

        self.reversing ={}
        self.reversing["speed"] = rospy.get_param("~speed", speed)
        self.reversing["duration"] = rospy.get_param("~duration", duration)
        self.reversing["distance"] = rospy.get_param("~duration", distance)
        self.reversing["mode"] = rospy.get_param("~duration", mode)

    def respawn(self):
        if self.reversing["mode"] == "timed":
            self.reverse_time(self.reversing["duration"], self.reversing["speed"])
        else:  # odomed
            self.reverse_tf(self.reversing["distance"], self.reversing["speed"])


if __name__ == '__main__':
    try:
        rev = Reversing(mode="timed")
        rev.respawn()
    except rospy.ROSInterruptException:
        pass
