#!/usr/bin/env python

""" movebase scouting

    whenever pose or direction request of target of interest is unknown, scout around map
    scouting strategy:
    create spiral waypoints from outer toward the center of map
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
import math
# from math import radians, pi, sin, cos, tan, ceil
from move_base_util import MoveBaseUtil

class Scout(MoveBaseUtil):
    # initialize boat pose param
    x0, y0, z0, roll0, pitch0, yaw0 = 0, 0, 0, 0, 0, 0

    def __init__(self, nodename):
        MoveBaseUtil.__init__(self, nodename)

        # get boat position, one time only
        self.odom_received = False
        rospy.wait_for_message("/odom", Odometry)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size = 50)

        # information about map, length (X), width (Y), position of the center wrt boat: boat-center
        # TODO (1) map center to absolute
        #      (2) get parameters by rospy.get_param()
        self.map_info = {"l":60, "w":60, "center_x": 30, "center_y":30}
        # set the offset distance from border
        offset=3

        while not self.odom_received:
            rospy.sleep(1)

        # create waypoints
        waypoints = self.scout_waypoints(self.map_info["l"], self.map_info["w"], self.map_info["center_x"], self.map_info["center_y"], offset)

        # Set a visualization marker at each waypoint
        for waypoint in waypoints:
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)

        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        # Initialize a counter to track waypoints
        i = 0

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
            self.move(goal)

            i += 1
        else:  # escape constant forward and continue to the next waypoint
            rospy.loginfo("Navigation test finished.")
            pass


    def scout_waypoints(self, map_x, map_y, center_x, center_y, offset):

        dis_x = self.x0 - center_x
        dis_y = self.y0 - center_y

        sign_x = math.copysign(1, dis_x)
        sign_y = math.copysign(1, dis_y)


        # Create a list to hold the target quaternions (orientations)
        corners = list()

        if math.floor(map_x / offset) < math.floor(map_y / offset):
            N = math.floor(map_x / (2 * offset)) - 1
        else:
            N = math.floor(map_y / (2 * offset)) - 1
        N = int(N)

        corners.append(Point(self.x0, self.y0, 0))

        # waypoints for the corners
        for i in range(N):
            corners.append(Point(center_x + sign_x * (N - i) * offset,
                           center_y + sign_y * (N - i) * offset, 0))
            corners.append(Point(center_x + sign_x * (N - i) * offset,
                           center_y - sign_y * (N - i) * offset, 0))
            corners.append(Point(center_x - sign_x * (N - i) * offset,
                           center_y - sign_y * (N - i) * offset, 0))
            corners.append(Point(center_x - sign_x * (N - i) * offset,
                           center_y + sign_y * (N - i - 1) * offset, 0))

        corners.append(Point(center_x, center_y, 0))

        # waypoints that joins corners
        waypoints=list()
        for i in range(N - 1):
            waypoints=self.straight_waypoints(corners[i], corners[i+1], waypoints)

        # return the resultant waypoints
        return waypoints

    def straight_waypoints(self, start, end, waypoints):

        # Create a list to hold the target quaternions (orientations)
        quaternions = list()

        #stores number of waypoints
        N=math.ceil(math.sqrt((end.x - start.x) ** 2 + (end.y - start.y) ** 2) / 5)
        N = int(N)


        # Then convert the angles to quaternions, all have the same heading angles
        for i in range(N):
            q_angle = quaternion_from_euler(0, 0,
                                            math.atan2((end.y - start.y),(end.x - start.x)))
            q = Quaternion(*q_angle)
            quaternions.append(q)

        catersian_x = [start.x + i * (end.x - start.x) / N
                       for i in range(N)]
        catersian_y = [start.y + i * (end.y - start.y) / N
                       for i in range(0,N)]

        # Append the waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        for i in range(N):
            waypoints.append(Pose(Point(catersian_x[i], catersian_y[i], 0.0),
                quaternions[i]))
            # return the resultant waypoints
        return waypoints

    def odom_callback(self, msg):
        """ call back to subscribe, get odometry data:
        pose and orientation of the current boat,
        suffix 0 is for origin """
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        self.z0 = msg.pose.pose.position.z
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.roll0, self.pitch0, self.yaw0 = euler_from_quaternion((x, y, z, w))
        self.odom_received = True
        # rospy.loginfo([self.x0, self.y0, self.z0])


if __name__ == '__main__':
    try:
        Scout("scout_test")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test aborted.")
