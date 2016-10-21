#!/usr/bin/env python

""" movebase zigzag

    do a zigzag scouting across the map


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

class Zigzag(object):
    # initialize boat pose param
    x0, y0, z0, roll0, pitch0, yaw0 = 0, 0, 0, 0, 0, 0

    def __init__(self):
        rospy.init_node('zigzag_test', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        # get boat position, one time only
        self.odom_received = False
        rospy.wait_for_message("/odom", Odometry)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size = 50)

        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # information about map, length (X), width (Y), position of the initial point
        self.map_info = {"l":60, "w":60}
	
	# set the half period and half amplitude      
	hp=10; 
        an=5;
	offset=5;

        while not self.odom_received:
            rospy.sleep(1)

        # create waypoints
        waypoints = self.create_waypoints(hp, an, offset)
	print type(waypoints)

        # Initialize the visualization markers for RViz
        self.init_markers()

        # Set a visualization marker at each waypoint
        for waypoint in waypoints:
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        # Initialize a counter to track waypoints
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
            self.move(goal)

            i += 1
        else:  # escape constant forward and continue to the next waypoint
            pass

    def create_waypoints(self, hp, an, offset):
	
	map_x=self.map_info["l"]
	map_y=self.map_info["w"]
	
	mid_y=floor(map_y/2)

	#calculate the number of tri(half way)
        N=floor((map_x-2*offset)/hp)
	N=int(N)

        # Create a list to hold the target points
        vertex = list()
	quaternions = list()
  

	vertex.append(Point(offset, mid_y, 0))
	q_angle = quaternion_from_euler(0, 0, -0.5*pi)
        q = Quaternion(*q_angle)
        quaternions.append(q)

	for i in range(0, N):	
	    vertex.append(Point(offset+i*hp+(hp/2), mid_y+an*((-1)**i), 0))
	    q_angle = quaternion_from_euler(0, 0, 0)
            q = Quaternion(*q_angle)
            quaternions.append(q)

	    vertex.append(Point(offset+(i+1)*hp, mid_y, 0))
	    q_angle = quaternion_from_euler(0, 0, -0.5*pi*(-1)**i)
            q = Quaternion(*q_angle)
            quaternions.append(q)

    	for i in range(0, N):
            vertex.append(Point(offset+(N-i)*hp-(hp/2), mid_y-an*((-1)**i), 0))
	    q_angle = quaternion_from_euler(0, 0, -pi)
            q = Quaternion(*q_angle)
            quaternions.append(q)

            vertex.append(Point(offset+(N-i-1)*hp, mid_y, 0))
	    q_angle = quaternion_from_euler(0, 0, 0.5*pi*(-1)**i)
            q = Quaternion(*q_angle)
            quaternions.append(q)

	waypoints=list()
	for i in range(0, len(vertex)):
            waypoints.append(Pose(vertex[i], quaternions[i]))
	
        # return the resultant waypoints
    	return waypoints 

    def move(self, goal):
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)

        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60 * 1))

        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")

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
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']

        self.markers.header.frame_id = 'odom'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

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

    def roi_callback(self, msg):
        """ from roi, get the relative distance and heading from the boat
        to the buoy marker """
        pass

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Zigzag()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
