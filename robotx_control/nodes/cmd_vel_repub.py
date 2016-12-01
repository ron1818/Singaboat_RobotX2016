#!/usr/bin/env python
# ren ye 20160517
# refernence:
# http://answers.ros.org/question/29706/twist-message-example-and-cmd_vel/
# http://blog.csdn.net/heyijia0327/article/details/41823809

# changelog
# @2016-11-01 use arduino + rosserial to drive the esc
# @2016-11-21 migrate to robotx

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msg.msg import Imu
from nav_msgs.msg import Odometry
# from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal  # , MoveBaseGoal
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from robotx_control.cfg import CmdVelPID


class Cmd_Vel_Repub(object):
    linear_kp, linear_ki, linear_kd,\
        angular_kp, angular_ki, angular_kd = 1, 1, 1, 1, 1, 1
    anglar_z, linear_x = 0, 0
    linear_acceleration_x, linear_acceleration_y, angular_acceleration_z = 0, 0, 0
    odom_x, odom_y, odom_yaw = 0, 0, 0
    goal_x, goal_y, goal_yaw = 0, 0, 0

    def __init__(self):
        rospy.init_node('cmd_vel_repub', anonymous=True)
        r = rospy.Rate(10)
        rospy.Subscriber("cmd_vel", Twist, callback=self.cmd_vel_callback, queue_size=10)
        rospy.Subscriber("imu/data", Imu, callback=self.imu_callback, queue_size=10)
        rospy.Subscriber("odom", Odometry, callback=self.odom_callback, queue_size=10)
        rospy.Subscriber("move_base/goal", MoveBaseActionGoal, callback=self.goal_callback, queue_size=10)
        self.srv = Server(CmdVelPID, self.dynamic_callback)
        # rospy.spin()
        cmd_vel_repub = rospy.Publisher("cmd_vel_filtered", Twist, queue_size=10)
        pid_cmd_vel_msg = Twist()
        while not rospy.is_shutdown():
            pid_cmd_vel_msg.linear.x, pid_cmd_vel_msg.angular_z = self.pid()
            cmd_vel_repub.publish(pid_cmd_vel_msg)
            r.sleep()

    def pid(self):
        """ do pid control here """
        # blahblahblah
        return (pid_linear_x, pid_angular_z)

    def dynamic_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: \
        {linear_kp}, {linear_ki}, {linear_kd}, \
        {angular_kp}, {angular_ki}, {angular_kd} """.format(**config))

        self.linear_kp = config["linear_kp"]
        self.linear_ki = config["linear_ki"]
        self.linear_kd = config["linear_kd"]
        self.angular_kp = config["angular_kp"]
        self.angular_ki = config["angular_ki"]
        self.angular_kd = config["angular_kd"]
        return config

    def cmd_vel_callback(self, msg):
        """ callback the subscribe, get Twist data """
        self.anglar_z = msg.angular.z  # rotation
        self.linear_x = msg.linear.x  # linear

    def imu_callback(self, msg):
        """ callback the subscribe, get imu data """
        self.linear_acceleration_x = msg.linear_acceleration.x
        self.linear_acceleration_y = msg.linear_acceleration.y
        self.angular_velocity_z = msg.angular_velocity.z

    def odom_callback(self, msg):
        """ callback the subscribe, get pose data """
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _, _, self.odom_yaw = euler_from_quaternion((x, y, z, w))

    def goal_callback(self, msg):
        self.goal_x = msg.goal.pose.position.x
        self.goal_y = msg.goal.pose.position.y
        x = msg.goal.pose.orientation.x
        y = msg.goal.pose.orientation.y
        z = msg.goal.pose.orientation.z
        w = msg.goal.pose.orientation.w
        _, _, self.goal_yaw = euler_from_quaternion((x, y, z, w))


if __name__ == '__main__':
    try:
        cmd_vel_repub = Cmd_Vel_Repub()

    except KeyboardInterrupt, rospy.ROSInterruptException:
        pass
