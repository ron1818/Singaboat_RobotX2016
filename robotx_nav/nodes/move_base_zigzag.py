#!/usr/bin/env python

""" movebase zigzag

    do a zigzag scouting across the map
    reinaldo
    2016-10-20
    # changelog:
    @2016-10-19: class inheriate from movebase util
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


class Zigzag(MoveBaseUtil):
    # initialize boat pose param
    x0, y0, z0, roll0, pitch0, yaw0 = 0, 0, 0, 0, 0, 0

    def __init__(self, nodename, is_newnode=True, quadrant=1, map_length=10, map_width=10, half_period=10, half_amplitude=10, offset=0):
        MoveBaseUtil.__init__(self, nodename, is_newnode)


	if quadrant is not None:
	    self.quadrant = quadrant
	else:
	    self.quadrant = rospy.get_param("~quadrant", quadrant)

        self.map_length = map_length
        self.map_width = map_width
        self.map_half_period = half_period
        self.map_half_amplitude = half_amplitude
        self.map_offset = offset


	if quadrant is not None:
	    self.respawn()


    def respawn(self, quadrant)
	self.quadrant=quadrant

        # assumption point 0,0 is the left-bottom of map
        if self.quadrant == 1:
            # create waypoints for quadrant 1
                waypoints = self.create_waypoints(self.map_half_period, self.map_half_amplitude, self.map_offset, self.map_length / 2, self.map_width / 2, self.map_length / 2, self.map_width / 2)
        elif self.quadrant == 2:
            # create waypoints for quadrant 2
                waypoints = self.create_waypoints(self.map_half_period, self.map_half_amplitude, self.map_offset, self.map_length / 2, self.map_width / 2, 0, self.map_width / 2)
        elif self.quadrant == 3:
            # create waypoints for quadrant 3
                waypoints = self.create_waypoints(self.map_half_period, self.map_half_amplitude, self.map_offset, self.map_length / 2, self.map_width / 2, 0, 0)
        elif self.quadrant == 4:
            # create waypoints for quadrant 4
                waypoints = self.create_waypoints(self.map_half_period, self.map_half_amplitude, self.map_offset, self.map_length / 2, self.map_width / 2, self.map_length / 2, 0)
        else:  # e.g. 0
            # create waypoints for the whole map
                waypoints = self.create_waypoints(self.map_half_period, self.map_half_amplitude, self.map_offset, self.map_length, self.map_width, 0, 0)

        # Initialize the visualization markers for RViz
        self.init_markers()

        # Set a visualization marker at each waypoint
        for waypoint in waypoints:
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)

        i = 0

        # Cycle through the waypoints
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
            self.move(goal, 1, 2)

            i += 1
        else:  # escape constant forward and continue to the next waypoint
            pass

    def create_waypoints(self, hp, an, offset, map_x, map_y, init_x, init_y):

        mid_y=floor(map_y / 2)

        #calculate the number of tri(half way)
        N = floor((map_x - 2 * offset) / hp)
        N = int(N)

        # Create a list to hold the target points
        vertex = list()
        quaternions = list()

        vertex.append(Point(init_x + offset, init_y + mid_y, 0))
        q_angle = quaternion_from_euler(0, 0, -0.5 * pi)
        q = Quaternion(*q_angle)
        quaternions.append(q)

        for i in range(0, N):
            vertex.append(Point(init_x + offset + i * hp + (hp / 2),
                          init_y + mid_y + an * ((-1) ** i), 0))
            q_angle = quaternion_from_euler(0, 0, 0)
            q = Quaternion(*q_angle)
            quaternions.append(q)

            vertex.append(Point(init_x + offset + (i + 1) * hp, init_y + mid_y, 0))
            q_angle = quaternion_from_euler(0, 0, -0.5 * pi * (-1) ** i)
            q = Quaternion(*q_angle)
            quaternions.append(q)

        for i in range(0, N):
            vertex.append(Point(init_x + offset + (N - i) * hp - (hp / 2),
                          init_y + mid_y - an * ((-1) ** i), 0))
            q_angle = quaternion_from_euler(0, 0, -pi)
            q = Quaternion(*q_angle)
            quaternions.append(q)

            vertex.append(Point(init_x + offset + (N - i - 1) * hp, init_y + mid_y, 0))
            q_angle = quaternion_from_euler(0, 0, 0.5 * pi * (-1) ** i)
            q = Quaternion(*q_angle)
            quaternions.append(q)

        waypoints = list()
        for i in range(0, len(vertex)):
            waypoints.append(Pose(vertex[i], quaternions[i]))

        # return the resultant waypoints
        return waypoints

if __name__ == '__main__':
    try:

        Zigzag(nodename="zigzag_test")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
