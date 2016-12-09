#!/usr/bin/env python
# license removed for brevity
"""
    move to a point without considering orientation
    reinaldo
    3-11-2016
    # changelog:
    @2016-10-20: class inheriate from movebase util
    @2016-12-01: split respawn function, assign new gps points

"""

import time
import rospy
import actionlib
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from math import radians, pi, sin, cos, atan2, floor, ceil, sqrt
from move_base_util import MoveBaseUtil
# from move_base_forward import Forward


class MoveToGeo(MoveBaseUtil):

    def __init__(self, nodename, is_newnode=True, target=None):
        MoveBaseUtil.__init__(self, nodename, is_newnode)

        # set the distance between waypoints
        self.geo = {}
        if target is not None:
            self.geo["target"] = [rospy.get_param("~latitude", target[0]),
                                 rospy.get_param("~longitude", target[1])]
            self.geo["goal_heading"] = rospy.get_param("~heading", target[2])
        else:  # must be updated in the self.respawn
            self.geo["target"] = [0, 0]
            self.geo["goal_heading"] = 0

        self.fix_received = False
        rospy.wait_for_message("/navsat/fix", NavSatFix)
        rospy.Subscriber("/navsat/fix", NavSatFix, self.navsat_fix_callback, queue_size = 50)
        while not self.fix_received:
            rospy.sleep(1)

        if target is not None:  # then we can call self.respawn for one time job
            self.respawn(None)  # without feeding new data

        ##### preparation stage finished #####
    def respawn(self, target=None):
        """ get a target and spawn a waypoint marker """
        # overwrite previous target
        if target is not None:
            self.geo["target"] = [rospy.get_param("~latitude", target[0]),
                                 rospy.get_param("~longitude", target[1])]
            self.geo["goal_heading"] = rospy.get_param("~heading", target[2])
        print self.target_lat, self.target_lon

        # create waypoint
        waypoint = self.create_waypoint()

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

    def create_waypoint(self):
        """ create waypoint from target lat/lon and current lat/lon """
        # rospy.loginfo("current position: " + str(self.lat) + ", " + str(self.lon))
        # rospy.loginfo("target position: " + str(self.target_lat) + ", " + str(self.target_lon))

        self.geo["translation"], self.geo["heading"] = \
            self.convert_gps_to_absolute(self.geo["target"][0], self.geo["target"][1])

        print self.geo["translation"], self.geo["heading"]

        # create waypoint
        q_angle = quaternion_from_euler(0, 0, self.geo["goal_heading"])
        quaternion = Quaternion(*q_angle)

        catersian_x, catersian_y = self.geo["translation"][0], self.geo["translation"][1]

        waypoint = Pose(Point(catersian_x, catersian_y, 0),
                        quaternion)

        self.markers.points.append(waypoint.position)

        return waypoint


if __name__ == '__main__':
    try:
        # MoveToGeo(nodename="movetogeo_test", target_lat=1.3489079, target_lon=103.6867139)

        # MoveToGeo(nodename="movetogeo_test", target_geo=(1.345124, 103.684729, 1.57))
        # target_geo = (1.345124, 103.684729, 1.57)
        # target_geo = (1.3451079, 103.6847139, 0)
        target_geo = (1.344423, 103.684952, 0)
        gps_waypoint = MoveToGeo(nodename="movetogeo_test", target=target_geo)
        # gps_waypoint.respawn(target_geo)
        time.sleep(2)
        print "next point"
        target_geo = (1.344469, 103.684666, 0)
        gps_waypoint.respawn(target_geo)
        time.sleep(2)
        print "next point"
        target_geo = (1.344716, 103.684909, 0)
        gps_waypoint.respawn(target_geo)
    except rospy.ROSInterruptException:
        pass
