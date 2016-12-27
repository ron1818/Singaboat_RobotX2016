#!/usr/bin/env python

""" move base utils
    modified from movebasesquare
    ren ye 2016-10-19
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from geographiclib.geodesic import Geodesic
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker
from math import radians, pi, sin, cos, sqrt


class MoveBaseUtil():
    x0, y0, yaw0 = 0, 0, 0
    lat, lon = 0, 0
    cancel_id = ""

    def __init__(self, nodename="nav_test", is_newnode=True):
        if is_newnode:
            rospy.init_node(nodename, anonymous=False)
            rate = rospy.Rate(10)
        else:
            rate = None

        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.fixed_frame = rospy.get_param("~fixed_frame", "map")
        # tf_listener
        self.tf_listener = tf.TransformListener()

        #rospy.on_shutdown(self.shutdown)

        # * get parameters
        # * Create a list to hold the target quaternions (orientations)
        # * Create a list to hold the waypoint poses
        # * create angles
        # * convert the angles to quaternions
        # * Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.

        # Initialize the visualization markers for RViz
        self.init_markers()

        self.odom_received = False
        # rospy.wait_for_message("/odom", Odometry)
        # rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=50)
        rospy.wait_for_message("/odometry/filtered/global", Odometry)
        rospy.Subscriber("/odometry/filtered/global", Odometry, self.odom_callback, queue_size=50)
        while not self.odom_received:
            rospy.sleep(1)

        rospy.Subscriber("move_base/cancel", GoalID, self.cancel_callback, queue_size=5)
        # * Set a visualization marker at each waypoint

        # * Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        # self.cmd_vel_pub = rospy.Publisher('move_base_cmd_vel', Twist, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # * Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # * Wait 60 seconds for the action server to become available
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        # * Cycle through the four waypoints

    def get_tf(self, fixed_frame, base_frame):
        """ transform from base_link to map """
        trans_received = False
        while not trans_received:
            try:
                (trans, rot) = self.tf_listener.lookupTransform(fixed_frame,
                                                                base_frame,
                                                                rospy.Time(0))
                trans_received = True
                return (Point(*trans), Quaternion(*rot))
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                pass

    def transform_tf(self, x_target_base, y_target_base, yaw_target_base, fixed_frame, base_frame):
        """ get the (x, y) wrt fixed frame from (x, y) wrt base frame"""
        # (x, y, yaw) of the base frame wrt fixed frame
        (trans, rot) = self.get_tf(fixed_frame, base_frame)
        x_base_fixed, y_base_fixed = trans.x, trans.y
        _, _, yaw_base_fixed = tf.transformations.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
        # get the point wrt fixed
        # final vector = fixed vector + rot_mat * base vector
        x_target_fixed, y_target_fixed = x_base_fixed + \
            cos(yaw_base_fixed) * x_target_base - sin(yaw_base_fixed) * y_target_base, \
                y_base_fixed + \
                    sin(yaw_base_fixed) * x_target_base + cos(yaw_base_fixed) * y_target_base
        yaw_target_fixed = yaw_target_base + rot

        return [x_target_fixed, y_target_fixed, yaw_target_fixed]

    def convert_gps_to_absolute(self, lat, lon):
        """ get current gps point of the boat,
            calculate the distance and heading to the target point
            remap to map frame """

        # calculate distance and azimuth (angle between distance and north)
        result = Geodesic.WGS84.Inverse(self.lat, self.lon, lat, lon)
        r = result['s12']
        azi = result['azi1'] * pi / 180.0
        theta = pi / 2 - azi  # wrt map's x axis
        # print "r and theta", r, theta

        center = [self.x0 + r * cos(theta), self.y0 + r * sin(theta), 0]
        heading = theta
        return [center, heading]

    def navsat_fix_callback(self, msg):
        """ callback navsat """
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.fix_received = True

    def odom_callback(self, msg):
        trans, rot = self.get_tf("map", "base_link")
        self.x0 = trans.x
        self.y0 = trans.y
        _, _, self.yaw0 = euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
        self.odom_received=True
        # """ call back to subscribe, get odometry data:
        # pose and orientation of the current boat,
        # suffix 0 is for origin """
        # x0 = msg.pose.pose.position.x
        # y0 = msg.pose.pose.position.y
        # # self.x0 = msg.pose.pose.position.x
        # # self.y0 = msg.pose.pose.position.y
        # x = msg.pose.pose.orientation.x
        # y = msg.pose.pose.orientation.y
        # z = msg.pose.pose.orientation.z
        # w = msg.pose.pose.orientation.w
        # # _, _, self.yaw0 = euler_from_quaternion((x, y, z, w))
        # _, _, yaw0 = euler_from_quaternion((x, y, z, w))
        # # get odom to map transform
        # self.x0, self.y0, self.yaw0 = self.transform_tf(x0, y0, yaw0, "map", "odom")

        # rospy.loginfo([self.x0, self.y0, self.yaw0])

    def cancel_callback(self, msg):
        self.cancel_id = msg.id
        print self.cancel_id
        # rospy.loginfo(self.cancel_id)

    def convert_relative_to_absolute(self, coordinate):
        """ boat's tf is base_link
        target is polar (r, theta) wrt base_link
        need to spawn waypoint (x1, y1) at map
        1. calculate target (xtb, ytb) wrt base_link by trignometry
        2. tf transfrom from base_link to map
        3. calculate target (x1, y1) wrt map by vector calculus:
            (x1, y1) = (xb, yb) + rot_mat*(xtb, ytb)
            where rot_mat = [cos theta, -sin theta; sin theta, cos theta]
        """

        # wrt base_link
        # theta is the angle between base_link's x axis and r
        # print "x0, y0, yaw0", self.x0, self.y0, self.yaw0
        r, theta = coordinate
        # print r, theta
        x_target_base, y_target_base = r * cos(theta), r * sin(theta)

        # print "target base", x_target_base, y_target_base

        x_target_rot, y_target_rot = \
            cos(self.yaw0) * x_target_base - sin(self.yaw0) * y_target_base, \
                sin(self.yaw0) * x_target_base + cos(self.yaw0) * y_target_base
        heading = theta + self.yaw0
        center = [self.x0 + x_target_rot, self.y0 + y_target_rot, 0]
        # print "target rot", x_target_rot, y_target_rot
        return [center, heading]

    def move(self, goal, mode, mode_param):
        """ mode1: continuous movement function, mode_param is the distance from goal that will set the next goal
            mode2: stop and rotate mode, mode_param is rotational angle in rad
            mode3: normal stop in each waypoint mode, mode_param is unused """

        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)
        finished_within_time = True
        go_to_next = False

        if mode == 1:  # continuous movement function, mode_param is the distance from goal that will set the next goal
            # (trans, _) = self.get_tf()
            while sqrt((self.x0 - goal.target_pose.pose.position.x) ** 2 +
                       (self.y0 - goal.target_pose.pose.position.y) ** 2) > mode_param:


                rospy.sleep(rospy.Duration(1))

                # (trans, _) = self.get_tf()
            go_to_next = True

        elif mode == 2:  # stop and rotate mode, mode_param is rotational angle in rad
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(40 * 1))
            self.rotation(mode_param)
            self.rotation(-2 * mode_param)
            self.rotation(mode_param)

        else:  # normal stop in each waypoint mode, mode_param is unused

            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60 * 1))


        # If we don't get there in time, abort the goal
        if not finished_within_time or go_to_next:
            self.move_base.cancel_goal()
            rospy.loginfo("Goal cancelled, next...")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")

    def rotation(self, ang):

        rate = rospy.Rate(10)
        an_vel = 0.1
        duration = ang / an_vel
        msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, an_vel))

        rate.sleep()
        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            if (current_time - start_time) > duration:
                self.cmd_vel_pub.publish(Twist(Vector3(0, 0.0, 0.0), Vector3(0.0, 0.0, -2 * an_vel)))
                self.cmd_vel_pub.publish(Twist())
                break
            else:
                self.cmd_vel_pub.publish(msg)
            rate.sleep()

    def reverse_tf(self, distance=5, speed=-1):
        """ reverse to certain distance """
        rate = rospy.Rate(10)
        linear_speed = speed
        if linear_speed > 0:
            linear_speed = -1 * linear_speed

        move_cmd = Twist()
        # Set the movement command to forward motion
        move_cmd.linear.x = linear_speed
        # Get the starting position values
        # (position, rotation) = self.get_tf()

        # x_start = position.x
        # y_start = position.y
        rate.sleep()
        x_start, y_start = self.x0, self.y0
        print x_start, y_start

        # Keep track of the distance traveled
        d = 0

        # Enter the loop to move along a side
        while d < distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            self.cmd_vel_pub.publish(move_cmd)

            rate.sleep()

            # Get the current position
            # (position, rotation) = self.get_tf()

            # Compute the Euclidean distance from the start
            d = sqrt(pow((self.x0 - x_start), 2) +
                     pow((self.y0 - y_start), 2))
            # print d

        # Stop the robot before the rotation
        move_cmd = Twist()
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)

    def reverse_time(self, duration=5, speed=-1):
        """ full reverse with a duration """
        rate = rospy.Rate(10)

        msg = Twist(Vector3(speed, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

        rate.sleep()
        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            if (current_time - start_time) > duration:
                self.cmd_vel_pub.publish(Twist())
                break
            else:
                self.cmd_vel_pub.publish(msg)
            rate.sleep()


    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0  # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}

        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)

        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        # self.markers.type = Marker.ARROW
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.scale.z = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']

        self.markers.header.frame_id = 'odom'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == "__main__":
    util = MoveBaseUtil()
    util.reverse_tf()
