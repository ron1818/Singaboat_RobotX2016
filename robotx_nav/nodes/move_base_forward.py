#!/usr/bin/env python

""" movebase constant heading

    Command a robot to move forward to a goal

    borrowed from rbx1 move base square

    Ren Ye
    2016-09-30

    constant heading behavior
    reinaldo
    2016-10-02
    # changelog:
    @2016-10-19: class inheriate from movebase util

"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import RegionOfInterest, CameraInfo
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker
from math import ceil, atan2, sqrt
from move_base_util import MoveBaseUtil


class Forward(MoveBaseUtil):
    # initialize boat pose param
    x0, y0, z0, roll0, pitch0, yaw0 = 0, 0, 0, 0, 0, 0

    def __init__(self, nodename, is_newnode=True, target=[20,0,0], waypoint_separation=5, is_relative=True):
        MoveBaseUtil.__init__(self, nodename, is_newnode)

        self.forward = {}
        if target is not None:
            self.target = Point(rospy.get_param("~target_x", target[0]), rospy.get_param("~target_y", target[1]), 0.0)
        else:  # must be updatged in the self.respawn
            self.target = Point(0, 0, 0)

        self.mode = rospy.get_param("~mode", 1)
        self.mode_param = rospy.get_param("~mode_param", 2)
        self.forward["waypoint_separation"] = rospy.get_param("~waypoint_separation", waypoint_separation)
        self.forward["is_relative"] = rospy.get_param("~is_relative", is_relative)

        if self.forward["is_relative"]:
            self.forward["translation"], self.forward["heading"] = self.convert_relative_to_absolute([self.target.x, self.target.y])
            # print self.forward["translation"], self.forward["heading"]
            self.forward["goal_distance"] = self.target.x
        else:
            # obtained from vision nodes, absolute catersian
            # but may be updated later, so need to callback
            self.forward["translation"] = (self.target.x, self.target.y, self.target.z)  # (x, y, 0)
            self.forward["goal_distance"] = sqrt((self.target.x - self.x0) ** 2 + (self.target.y - self.y0) ** 2)
            # heading from boat to center
            self.forward["heading"] = atan2(self.target.y - self.y0, self.target.x - self.x0)

    def respawn(self, target=None):
        if target is not None:
            self.target = Point(target[0], target[1], target[2])
            print self.target

        if self.forward["is_relative"]:
            self.forward["translation"], self.forward["heading"] = self.convert_relative_to_absolute([self.target.x, self.target.y])
            # print self.forward["translation"], self.forward["heading"]
            self.forward["goal_distance"] = self.target.x
        else:
            # obtained from vision nodes, absolute catersian
            # but may be updated later, so need to callback
            self.forward["translation"] = (self.target.x, self.target.y, self.target.z)  # (x, y, 0)
            self.forward["goal_distance"] = sqrt((self.target.x - self.x0) ** 2 + (self.target.y - self.y0) ** 2)
            # heading from boat to center
            self.forward["heading"] = atan2(self.target.y - self.y0, self.target.x - self.x0)

        # create waypoints
        waypoints = self.create_waypoints()

        # Set a visualization marker at each waypoint
        for waypoint in waypoints:
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)

        # Initialize a counter to track waypoints
        i = 1  # remove the first point

        # Cycle through the four waypoints
        while i < len(waypoints) and not rospy.is_shutdown():
            # Update the marker display
            self.marker_pub.publish(self.markers)

            # Intialize the waypoint goal
            goal = MoveBaseGoal()

            # Use the map frame to define goal poses
            goal.target_pose.header.frame_id = 'map'

            # Set the time stamp to "now"
            goal.target_pose.header.stamp = rospy.Time.now()

            # Set the goal pose to the i-th waypoint
            goal.target_pose.pose = waypoints[i]

            # Start the robot moving toward the goal
            self.move(goal, self.mode, self.mode_param)
            i += 1

        else:  # escape constant forward and continue to the next waypoint
            print "task finished"

    def create_waypoints(self):

        # Create a list to hold the target quaternions (orientations)
        quaternions = list()

        # First define the corner orientations as Euler angles
        # then calculate the position wrt to the center
        # need polar to catersian transform

        # stores number of waypoints
        N = ceil(self.forward["goal_distance"] / self.forward["waypoint_separation"])
        N = int(N)

        # Then convert the angles to quaternions, all have the same heading angles
        for i in range(N):
            q_angle = quaternion_from_euler(0, 0, self.forward["heading"])
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # Create a list to hold the waypoint poses
        waypoints = list()
        (trans, rot) = self.get_tf()
        catersian_x = [(N - i) * trans.x / N + i * self.forward["translation"][0] / N
                       for i in range(N)]
        catersian_y = [(N - i) * trans.y / N + i * self.forward["translation"][1] / N
                       for i in range(N)]

        # Append the waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        for i in range(N):
            waypoints.append(Pose(Point(catersian_x[i], catersian_y[i], 0.0),
                             quaternions[i]))
        # return the resultant waypoints
        return waypoints


if __name__ == '__main__':
    try:
        constant_heading = Forward(nodename="constantheading_test", is_relative=True)
        constant_heading.respawn(None)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
