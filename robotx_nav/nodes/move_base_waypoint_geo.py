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

    def __init__(self, nodename, target_geo, waypoint_distance=10):
        MoveBaseUtil.__init__(self, nodename)

        # set the distance between waypoints
        self.geo = {}
        self.target_lat = rospy.get_param("~target_latitude", target_geo[0])
        self.target_lon = rospy.get_param("~target_longitude", target_geo[1])
        self.geo["goal_heading"] = rospy.get_param("~target_heading", target_geo[2])
        self.geo["waypoint_distance"] = rospy.get_param("~waypoint_distance", waypoint_distance)

        rate = rospy.Rate(10)

        self.odom_received = False
        rospy.wait_for_message("/odom", Odometry)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size = 50)
        # rospy.wait_for_message("/odometry/filtered/global", Odometry)
        # rospy.Subscriber("/odometry/filtered/global", Odometry, self.odom_callback, queue_size = 50)
        while not self.odom_received:
            rospy.sleep(1)

            self.fix_received = False
            rospy.wait_for_message("/navsat/fix", NavSatFix)
            rospy.Subscriber("/navsat/fix", NavSatFix, self.navsat_fix_callback, queue_size = 50)
        while not self.fix_received:
            rospy.sleep(1)

        rospy.loginfo("org position: " + str(self.lat0) + ", " + str(self.lon0))
        rospy.loginfo("tar position: " + str(self.target_lat) + ", " + str(self.target_lon))

        # self.markers.points.append(waypoint.position)
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        waypoint = self.create_waypoint()
        rospy.loginfo("Waiting for move_base action server...")
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        # Update the marker display
        self.marker_pub.publish(self.markers)

        # Intialize the waypoint goal
        goal = MoveBaseGoal()

        # Use the map frame to define goal poses
        goal.target_pose.header.frame_id = 'map'

        # Set the time stamp to "now"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal pose to the i-th waypoint
        goal.target_pose.pose = waypoint

        # Start the robot moving toward the goal
        self.move(goal, mode=0, mode_param=3)
        # rospy.sleep(1.)
        # rospy.spin()


    def create_waypoint(self):
        """ create waypoint from target lat/lon and current lat/lon """
        rospy.loginfo("current position: " + str(self.lat0) + ", " + str(self.lon0))
        rospy.loginfo("target position: " + str(self.target_lat) + ", " + str(self.target_lon))

        result = Geodesic.WGS84.Inverse(self.lat0, self.lon0,
                                        self.target_lat, self.target_lon)
        self.geo["goal_distance"] = result['s12']  # distance from boat position to target

        azi = result['azi1'] * pi / 180
        print self.yaw0
        theta =pi / 2 - self.yaw0 - azi

        rospy.loginfo(str(self.geo["goal_distance"]) + "," + str(theta))

        self.geo["translation"], self.geo["heading"] = \
            self.convert_relative_to_absolute([self.x0, self.y0, self.yaw0], (self.geo["goal_distance"], theta))


        # create waypoint
        q_angle = quaternion_from_euler(0, 0, self.geo["goal_heading"])
        quaternion = Quaternion(*q_angle)

        catersian_x = self.x0 + self.geo["translation"][0]
        catersian_y = self.y0 + self.geo["translation"][1]

        waypoint = Pose(Point(catersian_x, catersian_y, 0),
                        quaternion)

        self.markers.points.append(waypoint.position)

        return waypoint


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
