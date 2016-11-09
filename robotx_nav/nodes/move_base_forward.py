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
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker
from math import radians, pi, sin, cos, tan, ceil, atan2, sqrt
from move_base_util import MoveBaseUtil

class Forward(MoveBaseUtil):
    # initialize boat pose param
    x0, y0, z0, roll0, pitch0, yaw0 = 0, 0, 0, 0, 0, 0

    def __init__(self, nodename, target):
        MoveBaseUtil.__init__(self, nodename)

        self.forward={}

        # get boat position, one time only
        self.odom_received = False
        rospy.wait_for_message("/odom", Odometry)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size = 50)

        while not self.odom_received:
            rospy.sleep(1)


	# set the distance between waypoints
   	self.forward["waypoint_distance"]=rospy.get_param("~waypoint_distance", 5)
        # check whether absolute or relative target
   	self.forward["is_relative"]=rospy.get_param("~is_relative", False)

        if self.forward["is_relative"]:
            self.forward["translation"], self.forward["heading"] = \
                    self.convert_relative_to_absolute([self.x0, self.y0, self.yaw0], target)



        else: # absolute
            # obtained from vision nodes, absolute catersian
            # but may be updated later, so need to callback
            self.forward["translation"] = (target.x, target.y, target.z)  # (x, y, 0)
            self.forward["goal_distance"]= sqrt((target.x - self.x0) ** 2 + (target.y - self.y0) ** 2)
            # heading from boat to center
            self.forward["heading"] = atan2(target.y - self.y0, target.x - self.x0)

        # create waypoints
        waypoints = self.create_waypoints()
        # print type(waypoints)

        ## Initialize the visualization markers for RViz
        # self.init_markers()

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
            self.move(goal, 1, 3)
            i += 1

        else:  # escape constant forward and continue to the next waypoint
            pass

    def create_waypoints(self):

        # Create a list to hold the target quaternions (orientations)
        quaternions = list()

        # First define the corner orientations as Euler angles
        # then calculate the position wrt to the center
        # need polar to catersian transform

	#stores number of waypoints
	N = ceil(self.forward["goal_distance"]/self.forward["waypoint_distance"])
        N = int(N)

        # Then convert the angles to quaternions, all have the same heading angles
        for i in range(N):
            q_angle = quaternion_from_euler(0, 0, self.forward["heading"])
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # Create a list to hold the waypoint poses
        waypoints = list()
        catersian_x = [(N - i) * self.x0 / N + i * self.forward["translation"][0] / N
                       for i in range(N)]
        catersian_y = [(N - i) * self.y0 / N + i * self.forward["translation"][1] / N
                       for i in range(N)]

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
        Forward(nodename="constantheading_test", target=Point(10, 20, 0))
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
