#!/usr/bin/env python
# license removed for brevity
"""
   station keeping
    @Weiwei
    2016-10-25

   corrected: reinaldo


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


class StationKeeping(MoveBaseUtil):

    def __init__(self, nodename, is_newnode=True, target=None, radius=2, duration=100):
        MoveBaseUtil.__init__(self, nodename, is_newnode=is_newnode)

        self.sk = {}
        if target is not None:  # a target point to hold the position
            self.sk["target"] = Twist(Point(rospy.get_param("~target_x", target[0]),
                                rospy.get_param("~target_y", target[1]), 0),
                                Point(0, 0, rospy.get_param("~angle", target[2])))
        else:  # hold the current boat's position
            self.sk["target"] = Twist(Point(self.x0, self.y0, 0),
                                Point(0, 0, self.yaw0))
        self.sk["radius"] = rospy.get_param("~radius", radius)
        self.sk["duration"] = rospy.get_param("~duration", duration)

        if target is not None:
            self.respawn(None)

    def respawn(self, target, radius, duration):
        if target is not None:  # a target point to hold the position
            self.sk["target"] = Twist(Point(target[0], target[1], 0), Point(0, 0, target[2]))
        if radius is not None:
            self.sk["radius"] = radius
        if duration is not None:
            self.sk["duration"] = duration

        q_angle = quaternion_from_euler(0, 0, self.sk["target"].angular.z)
        angle = Quaternion(*q_angle)
        station = Pose(self.sk["target"].linear, angle)

        p = Point()
        p = station.position
        self.markers.points.append(p)

        self.marker_pub.publish(self.markers)

        #get start time
        start_time = rospy.get_time()

        while ((rospy.get_time()-start_time < self.sk["duration"]) or not self.sk["duration"]) and not rospy.is_shutdown():
            if (sqrt((self.sk["target"].linear.x-self.x0)**2 + (self.sk["target"].linear.y-self.y0)**2) < self.sk["radius"]):
                self.cmd_vel_pub.publish(Twist())
                #rospy.loginfo("inside inner radius, no action")
            else:
                rospy.loginfo("outside radius")
                # Intialize the waypoint goal
                goal = MoveBaseGoal()

                # Use the map frame to define goal poses
                goal.target_pose.header.frame_id = 'map'

                # Set the time stamp to "now"
                goal.target_pose.header.stamp = rospy.Time.now()

                # Set the goal pose to the waypoint
                goal.target_pose.pose = station

                # Start the robot moving toward the goal

                self.move(goal, 0, 0)
                rospy.loginfo("goal sent")
        else:
            rospy.loginfo("station keep ends")


if __name__ == '__main__':
    try:
        StationKeeping("station_keeping_test")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        pass
