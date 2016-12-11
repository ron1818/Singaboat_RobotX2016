#!/usr/bin/env python
# license removed for brevity
"""
    Reinaldo 5-12-16
    Aim to a target by rotational pid controller when inside inner radius, and go back to station otherwise


"""

import rospy
import actionlib
import time
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

    def __init__(self, nodename, is_newnode=True, target=None, radius=2, duration=0, angle_tolerance=10*pi/180.0, box=[0,0,0]):
        MoveBaseUtil.__init__(self, nodename, is_newnode)

        if target is not None:  # shooting range's position
            self.target = Twist(Point(rospy.get_param("~station_x", target[0]), rospy.get_param("~station_y", target[1]), 0), Point(0, 0, rospy.get_param("~station_yaw", target[2])))
        else:  # use boat's current position as the shooting range's position
            self.target = Twist(Point(self.x0, self.y0, 0), Point(0, 0, self.yaw0))
        self.radius = radius
        self.duration = duration
        self.angle_tolerance = angle_tolerance
        self.box = box

        if self.target is not None: # onetime job
            self.respawn()

    def respawn(self, duration, box, target):
        self.box=box
        self.duration=duration
        self.target=target

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
                theta = atan2(self.box.y -self.y0, self.box.x - self.x0)
                if (abs(atan2(sin(theta - self.yaw0), cos(theta - self.yaw0))) > self.angle_tolerance):
                    print "correcting", theta , self.yaw0

                    aim_target=self.aim_to_box([self.box.x, self.box.y], 30) #recheck condition every 30s

                    rospy.sleep(1)

            else:
                rospy.loginfo("outside radius")
                # Intialize the waypoint goal
                aim_target.shutdown()

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

    def aim_to_box(self, target, duration):

        cmd_vel_pub= rospy.Publisher("cmd_vel", Twist, queue_size=10)
        pid_cmd_vel_msg = Twist()


        #initialise pid variables
        #angular
        self.Integrator_max_angular=500
        self.Integrator_min_angular=-500

        self.error_angular=0.0
        self.Derivator_angular=0.0
        self.Integrator_angular=0.0

        start_time = rospy.get_time()

        while not (rospy.get_time() - start_time) < duration:

            pid_cmd_vel_msg.angular.z= self.pid_angular(target)

            cmd_vel_pub.publish(pid_cmd_vel_msg)
            time.sleep(0.2)


    def pid_angular(self, target):
        #angular PID
        angle_error=math.atan2(target[1]-self.y0, target[0]-self.x0)-self.yaw0

        self.error_angular=atan2(sin(angle_error), cos(angle_error)) #trick to remap to -pi -
        self.P_value_angular=self.angular_kp*self.error_angular

        derivative_error=self.error_angular - self.Derivator_angular
        self.D_value_angular=self.angular_kd*atan2(sin(derivative_error), cos(derivative_error))
        self.Derivator_angular=self.error_angular

        self.Integrator_angular=self.Integrator_angular +self.error_angular

        if self.Integrator_angular > self.Integrator_max_angular:
            self.Integrator_angular=self.Integrator_max_angular
        elif self.Integrator_angular < self.Integrator_min_angular:
            self.Integrator_angular=self.Integrator_min_angular

        self.I_value_angular=self.Integrator_angular*self.angular_ki


        pid_angular_z=self.P_value_angular + self.I_value_angular + self.D_value_angular

        if pid_angular_z>self.angular_velocity_threshold:
            pid_angular_z=self.angular_velocity_threshold
        elif pid_angular_z<-self.angular_velocity_threshold:
            pid_angular_z=-self.angular_velocity_threshold

        return (pid_angular_z)


    def dynamic_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: \
         \ 
         {aim_angular_kp}, {aim_angular_ki}, {aim_angular_kd}, {aim_angular_velocity_threshold}  """.format(**config))

        self.angular_kp = config["aim_angular_kp"]
        self.angular_ki = config["aim_angular_ki"]
        self.angular_kd = config["aim_angular_kd"]

        self.angular_velocity_threshold=config["aim_angular_velocity_threshold"]
        return config


if __name__ == '__main__':
    try:

        Aiming("aiming_test")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        pass
y`