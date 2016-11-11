#!/usr/bin/env python
# license removed for brevity
"""
    move to a point without considering orientation
    reinaldo
    3-11-2016
    # changelog:
    @2016-10-20: class inheriate from movebase util

"""

import rospy
import actionlib
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geographiclib.geodesic import Geodesic
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from math import radians, pi, sin, cos, atan2, floor, ceil, sqrt
from move_base_util import MoveBaseUtil
# from move_base_forward import Forward

class MoveToGeo(MoveBaseUtil):
    # initialize boat pose param
    x0, y0, z0, roll0, pitch0, yaw0, lon0, lat0 = 0, 0, 0, 0, 0, 0, 0, 0

    def __init__(self, nodename, target_geo, waypoint_distance=5):
        MoveBaseUtil.__init__(self, nodename)  # , target=(0,0,0),
                         # waypoint_distance=waypoint_distance,
                         # is_relative=is_relative)
	# self.odom_frame = rospy.get_param("~odom_frame", "odom")
        # self.fixed_frame = rospy.get_param("~fixed_frame", "base_link")

        # set the distance between waypoints
        self.geo = {}
        self.geo["waypoint_distance"] = rospy.get_param("~waypoint_distance", waypoint_distance)


	self.odom_received = False
        rospy.wait_for_message("/odometry/filtered/global", Odometry)
        rospy.Subscriber("/odometry/filtered/global", Odometry, self.odom_callback, queue_size = 50)
        rospy.loginfo(self.odom_received)
        while not self.odom_received:
            rospy.sleep(1)

	self.fix_received = False
        rospy.wait_for_message("/navsat/fix", NavSatFix)
	rospy.Subscriber("/navsat/fix", NavSatFix, self.navsat_fix_callback, queue_size = 50)
        while not self.fix_received:
            rospy.sleep(1)

	# listener = tf.TransformListener()
	# try:
        #     (trans, rot) = listener.lookupTransform(self.fixed_frame, self.odom_frame, rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     pass


	#get current lat and lon of boat
        print self.lat0, self.lon0
        print target_geo[0], target_geo[1]
	result = Geodesic.WGS84.Inverse(self.lat0, self.lon0, target_geo[0], target_geo[1])
	d = result['s12']  # distance from boat position to target
        self.geo["goal_distance"] = d

	azi = result['azi1'] * pi / 180
        theta = self.yaw0 - (pi / 2 - azi)

	# phi=atan2(trans[1], trans[0]) #odom frame wrt to fixed frame (y faces North)
	#calculate angle wrt to local map
	# alpha=pi/2-theta-phi
        rospy.loginfo(str(d) + "," + str(theta))

        self.geo["translation"], self.geo["heading"] = \
            self.convert_relative_to_absolute([self.x0, self.y0, self.yaw0], (d, theta))

        # create waypoints
        waypoints = self.create_waypoints()
        # print type(waypoints)

        # Initialize the visualization markers for RViz
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

        # stores number of waypoints
        N = ceil(self.geo["goal_distance"] / self.geo["waypoint_distance"])
        N = int(N)

        # Then convert the angles to quaternions, all have the same heading angles
        for i in range(N):
            q_angle = quaternion_from_euler(0, 0, self.geo["heading"])
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # Create a list to hold the waypoint poses
        waypoints = list()
        catersian_x = [(N - i) * self.x0 / N + i * self.geo["translation"][0] / N
                       for i in range(N)]
        catersian_y = [(N - i) * self.y0 / N + i * self.geo["translation"][1] / N
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


    def navsat_fix_callback(self, msg):
	self.lat0 = msg.latitude
	self.lon0 = msg.longitude
        self.fix_received = True
        # rospy.loginfo(str(self.lat0) +"," + str(self.lon0))


if __name__ == '__main__':
    try:
        # MoveToGeo(nodename="movetogeo_test", target_lat=1.3489079, target_lon=103.6867139)
        # MoveToGeo(nodename="movetogeo_test", target_geo=(1.345124, 103.684729, 1.57))
        MoveToGeo(nodename="movetogeo_test", target_geo=(1.344452, 103.684460, 1.57))
    except rospy.ROSInterruptException:
        pass
