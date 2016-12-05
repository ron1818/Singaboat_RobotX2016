#!/usr/bin/env python
# license removed for brevity
"""
    move to a point without considering orientation
    reinaldo
    2016-10-20
    # changelog:
    @2016-10-20: class inheriate from movebase util

"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import RegionOfInterest, CameraInfo
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker
from math import radians, pi, sin, cos, atan2, floor, ceil, sqrt
from move_base_util import MoveBaseUtil

class MoveTo(MoveBaseUtil):
    # initialize boat pose param
    x0, y0, z0, roll0, pitch0, yaw0 = 0, 0, 0, 0, 0, 0

    def __init__(self, nodename, is_newnode=True, target=[0,10,1.57], is_relative=False):
        MoveBaseUtil.__init__(self, nodename, is_newnode)

        self.moveto={}
        if target is not None:
            self.moveto["target"] = Point(rospy.get_param("~target_x", target[0]),
                                          rospy.get_param("~target_y", target[1]),
                                          rospy.get_param("~heading", target[2]) )
        else:
            self.moveto["target"] = Point(0, 0, 0)

        self.moveto["is_relative"] = rospy.get_param("~is_relative", is_relative)

        if target is not None:
            self.respawn()


    def respawn(self, target=None):
        if target is not None:
            self.moveto["target"] = Point(target[0], target[1], target[2])

        print self.moveto["target"]

        if self.moveto["is_relative"]:
            position, heading = self.convert_relative_to_absolute([self.moveto["target"].x, self.moveto["target"].y])
            x, y, _ = position
        else:  # map frame
            x, y = self.moveto["target"].x, self.moveto["target"].y

        q_angle = quaternion_from_euler(0, 0, atan2(y-self.y0, x-self.x0))
        angle = Quaternion(*q_angle)

        waypoint = Pose(Point(x, y, 0), angle)

        # Set a visualization marker at each waypoint

        p = Point()
        p = waypoint.position
        self.markers.points.append(p)

        self.marker_pub.publish(self.markers)

        # Intialize the waypoint goal
        goal = MoveBaseGoal()

        # Use the map frame to define goal poses
        goal.target_pose.header.frame_id = 'map'

        # Set the time stamp to "now"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal pose to the i-th waypoint
        goal.target_pose.pose = waypoint

        # Start the robot moving toward the goal
        self.move(goal, 0, 0)


if __name__ == '__main__':
    try:
        MoveTo(nodename="moveto_waypoint")
    except rospy.ROSInterruptException:
        pass
