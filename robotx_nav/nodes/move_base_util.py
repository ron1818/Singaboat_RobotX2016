#!/usr/bin/env python

""" move base utils
    modified from movebasesquare
    ren ye 2016-10-19
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi, sin, cos, sqrt

class MoveBaseUtil():
    def __init__(self, nodename="nav_test"):
        rospy.init_node(nodename, anonymous=False)

        rospy.on_shutdown(self.shutdown)

        # * get parameters

        # * Create a list to hold the target quaternions (orientations)

        # * Create a list to hold the waypoint poses

        # * create angles
        # * convert the angles to quaternions


        # * Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.

        # Initialize the visualization markers for RViz
        self.init_markers()

        # * Set a visualization marker at each waypoint

        # * Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # * Subscribe to the move_base action server
        # self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # * Wait 60 seconds for the action server to become available
        # self.move_base.wait_for_server(rospy.Duration(60))

        # * Cycle through the four waypoints

    def convert_relative_to_absolute(self, boat, target):
        """ boat is catersian (x0, y0),
        target is polar (r, theta)
        and absolute is catersian (x1, y1) """
        r, theta = target
        x, y, yaw = boat
        heading = theta + (yaw - pi / 2)
        center = [x + r * cos(heading),
                  y + r * sin(heading),
                  0]

        return [center, heading]

    def move(self, goal, mode, mode_param):
            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal)
	  
            finished_within_time = 'true'
	    go_to_next= 'false'
           
  	    if mode==1: #continuous movement function, mode_param is the distance from goal that will set the next goal
		while sqrt((self.x0-goal.target_pose.pose.position.x)**2+(self.y0-goal.target_pose.pose.position.y)**2)>mode_param:
		    rospy.sleep(rospy.Duration(1))
		go_to_next='true'

	    elif mode==2: #stop and rotate mode, mode_param is rotational angle in rad
		finished_within_time = self.move_base.wait_for_result(rospy.Duration(40 * 1))
		self.rotation(mode_param)
		self.rotation(-2*mode_param)
		self.rotation(mode_param)		
	    else: #normal stop in each waypoint mode, mode_param is unused
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

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)
        an_vel=0.2
        duration=ang/an_vel;
        msg=Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, an_vel))

        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            current_time=rospy.get_time()
            if (current_time - start_time) > duration:
                pub.publish(Twist(Vector3(0, 0.0, 0.0), Vector3(0.0, 0.0, -2*an_vel)))
                rospy.sleep(0.3)
                pub.publish(Twist())
                break
            pub.publish(msg)
            rate.sleep()


    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
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
