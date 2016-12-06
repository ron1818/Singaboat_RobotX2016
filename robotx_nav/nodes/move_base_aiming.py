#!/usr/bin/env python
# license removed for brevity
"""
   station keeping
    @Weiwei
    2016-10-25

   corrected: reinaldo


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
from move_base_util import MoveBaseUtil


class Aiming(MoveBaseUtil):
    # initialize boat pose param
    # x0, y0, z0, roll0, pitch0, yaw0 = 0, 0, 0, 0, 0, 0

    def __init__(self, nodename, target=None, radius=2, duration=200, angle_tolerance=1*pi/180.0, box=[5,0,0]):
        MoveBaseUtil.__init__(self, nodename)

        if target is not None:  # shooting range's position
            self.target = Twist(Point(rospy.get_param("~station_x", target[0]), rospy.get_param("~station_y", target[1]), 0),
                                    Point(0, 0, rospy.get_param("~station_yaw", target[2])))
        else:  # use boat's current position as the shooting range's position
            self.target = Twist(Point(self.x0, self.y0, 0), Point(0, 0, self.yaw0))
        self.radius = rospy.get_param("~radius", radius)
        self.duration = rospy.get_param("~duration", duration)
        self.angle_tolerance = rospy.get_param("~angle_tolerance", angle_tolerance)
        self.box = Point(rospy.get_param("~box_x", box[0]), rospy.get_param("~box_y", box[1]), 0)

        # # get boat pose one time onl
        # self.odom_received = False
        # rospy.wait_for_message("/odom", Odometry)
        # rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=50)

        # while not self.odom_received:
        #     rospy.sleep(1)

        # # Subscribe to the move_base action server
        # self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # rospy.loginfo("Waiting for move_base action server...")

        # # Wait 60 seconds for the action server to become available
        # self.move_base.wait_for_server(rospy.Duration(60))

        # rospy.loginfo("Connected to move base server")
        # rospy.loginfo("Starting navigation test")

        q_angle = quaternion_from_euler(0, 0, self.target.angular.z)
        angle = Quaternion(*q_angle)
        station = Pose(self.target.linear, angle)

        p = Point()
        p = station.position
        self.markers.points.append(p)

        self.marker_pub.publish(self.markers)

        # get start time
        start_time = rospy.get_time()

        while (rospy.get_time() - start_time < self.duration) or not self.duration and not rospy.is_shutdown():
            if (sqrt((self.target.linear.x - self.x0)**2 + (self.target.linear.y - self.y0) ** 2) < self.radius):
                rospy.loginfo("inside inner radius, corrects orientation to face box")
                theta = atan2(self.box.y - self.y0, self.box.x - self.x0)
                if(abs(theta - self.yaw0) > self.angle_tolerance):
                    print "correcting", theta , self.yaw0
                    self.rotation(theta - self.yaw0)
                    rospy.sleep(1)

            else:
                rospy.loginfo("outside radius")
                # Intialize the waypoint goal
                goal = MoveBaseGoal()

                # Use the map frame to define goal poses
                goal.target_pose.header.frame_id = 'map'

                # Set the time stamp to "now"
                goal.target_pose.header.stamp = rospy.Time.now()

                # Set the goal pose to the waypoint
                goal.target_pose.pose = station

                # Start the robot moving toward the goal

                self.move(goal, 0, 0)
                rospy.loginfo("goal sent")

    # def odom_callback(self, msg):
    #     """ call back to subscribe, get odometry data:
    #     pose and orientation of the current boat,
    #     suffix 0 is for origin """
    #     self.x0 = msg.pose.pose.position.x
    #     self.y0 = msg.pose.pose.position.y
    #     self.z0 = msg.pose.pose.position.z
    #     x = msg.pose.pose.orientation.x
    #     y = msg.pose.pose.orientation.y
    #     z = msg.pose.pose.orientation.z
    #     w = msg.pose.pose.orientation.w
    #     self.roll0, self.pitch0, self.yaw0 = euler_from_quaternion((x, y, z, w))
    #     self.odom_received = True
    #     # rospy.loginfo([self.x0, self.y0, self.z0])


if __name__ == '__main__':
    try:

        Aiming("aiming_test")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        pass
