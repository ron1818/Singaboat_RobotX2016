#!/usr/bin/env python

""" movebase scouting

    whenever pose or direction request of target of interest is unknown, scout around map
    scouting strategy:
    create spiral waypoints from outer toward the center of map 


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
from math import radians, pi, sin, cos, tan, ceil

class Scout(object):
    # initialize boat pose param
    x0, y0, z0, roll0, pitch0, yaw0 = 0, 0, 0, 0, 0, 0

    def __init__(self):
        rospy.init_node('scout_test', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        # get boat position, one time only
        self.odom_received = False
        rospy.wait_for_message("/odom", Odometry)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size = 50)

        # information about map, length (X), width (Y), position of the center wrt boat: boat-center
        self.map_info = {"l":60, "w":60, "center_x": 10, "center_y":5}
	# set the offset distance from border         
	offset=3; 

        while not self.odom_received:
            rospy.sleep(1)

        # create waypoints
        waypoints = self.scout_waypoints(self, self.map_info["l"], self.map_info["w"], self.map_info["center_x"], self.map_info["center_y"], offset)

        # Initialize the visualization markers for RViz
        self.init_markers()

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
        while i < waypoints.len() and not rospy.is_shutdown():
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

    def scout_waypoints(self, map_x, map_y, center_x, center_y, offset):

	dis_x=self.x0-center_x
	dis_y=self.y0-center_y

	math.copysign(sign_x, dis_x)
	math.copysign(sign_y, dis_y)
	

        # Create a list to hold the target quaternions (orientations)
        corners = list()
  
	if math.floor(map_x/offset)<math.floor(map_y/offset):
	    N=math.floor(map_x/(2*offset))-1
	else:
	    N=math.floor(map_y/(2*offset))-1 
		
	corner.append(Point(self.x0, self.y0, 0))

	for i in  range(0,N):
	    corner.append(Point(center_x+sign_x*(N-i)*offset, center_y+sign_y*(N-i)*offset, 0))
	    corner.append(Point(center_x+sign_x*(N-i)*offset, center_y-sign_y*(N-i)*offset, 0))
      	    corner.append(Point(center_x-sign_x*(N-i)*offset, center_y-sign_y*(N-i)*offset, 0))
  	    corner.append(Point(center_x-sign_x*(N-i)*offset, center_y+sign_y*(N-i-1)*offset, 0))
	
	corner.append(Point(center_x, center_y, 0))

	waypoints=list()

	for i in range(0,N-1):
	    waypoints=straight_waypoints(self, corner[i], corner[i+1], waypoints)
	
        # return the resultant waypoints
        return waypoints

    def straight_waypoints(self, start, end, waypoints):
	
	# Create a list to hold the target quaternions (orientations)
        quaternions = list()

	#stores number of waypoints
	N=math.ceil(math.sqrt((end.x-start.x)**2+(end.y-start.y)**2)/5);
 
	
        # Then convert the angles to quaternions, all have the same heading angles
        for i in N:
            q_angle = quaternion_from_euler(0, 0, math.atan2((end.y-start.y),(end.x-start.x)))
            q = Quaternion(*q_angle)
            quaternions.append(q)

        catersian_x = [start.x+i*(end.x-start.x)/N
                       for i in range(0,N)]
        catersian_y = [start.y+i*(end.y-start.y)/N
                       for i in range(0,N)]

        # Append the waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        for i in N:
            waypoints.append(Pose(Point(catersian_x[i], catersian_y[i], 0.0),
                             quaternions[i]))
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
        Forward()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
