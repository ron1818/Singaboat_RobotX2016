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
from geographiclib.geodesic import Geodesic
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from math import radians, pi, sin, cos, atan2, floor, ceil, sqrt
from move_base_util import MoveBaseUtil
from robotx_nav import Forward

class MoveToGeo(MoveBaseUtil):
    # initialize boat pose param
    x0, y0, z0, roll0, pitch0, yaw0, lon0, lat0 = 0, 0, 0, 0, 0, 0, 0, 0

    def __init__(self, nodename, target_lat, target_lon):
        MoveBaseUtil.__init__(self, nodename)
	self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.fixed_frame = rospy.get_param("~fixed_frame", "base_link")
        
	self.odom_received = False
        rospy.wait_for_message("/odom", Odometry)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size = 50)
	rospy.Subscriber("/navsat/fix", NavSatFix, self.navsat_fix_callback, queue_size = 50)
	
	listener = tf.TransformListener()
	try:
            (trans,rot) = listener.lookupTransform(self.fixed_frame, self.odom_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

	
	#get current lat and lon of boat
	result = Geodesic.WGS84.Inverse(self.lat0, self.lon0, target_lat, target_lon)
	d=result['s12'] #distance from boat position to target
	
	theta=result['azi1']*pi/180
	phi=atan2(trans[1], trans[0]) #odom frame wrt to fixed frame (y faces North)
	#calculate angle wrt to local map
	alpha=pi/2-theta-phi
	
	
        Forward(nodename="moveto_test", target=Point(self.x0+d*sin(alpha),self.y0+d*cos(alpha),0))

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
	self.lat0=msg.latitude
	self.lon0=msg.longitude


if __name__ == '__main__':
    try:
        MoveToGeo(nodename="movetogeo_test", target_lat=1.3489079, target_lon=103.6867139)
    except rospy.ROSInterruptException:
        pass
