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
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
# from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal  # , MoveBaseGoal
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from robotx_control.cfg import CmdVelPID


class Cmd_Vel_Repub(object):
    linear_kp, linear_ki, linear_kd,\
        angular_kp, angular_ki, angular_kd = 1, 1, 1, 1, 1, 1
    angular_z, linear_x = 0, 0
    linear_acceleration_x, linear_acceleration_y, angular_acceleration_z = 0, 0, 0
    odom_x, odom_y, odom_yaw = 0, 0, 0
    goal_x, goal_y, goal_yaw = 0, 0, 0
    linear_threshold=5.0
    angular_threshold=1.0

    linear_velocity_threshold=3
    angular_velocity_threshold=1.5

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


        #initialise pid variables
        #linear
        self.Integrator_max_linear=500
        self.Integrator_min_linear=-500

        self.set_point_linear=0.0 #desired value, 
        self.error_linear=0.0
        self.Derivator_linear=0.0
        self.Integrator_linear=0.0
        #angular
        self.Integrator_max_angular=500
        self.Integrator_min_angular=-500

        self.set_point_angular=0.0
        self.error_angular=0.0
        self.Derivator_angular=0.0
        self.Integrator_angular=0.0



        while not rospy.is_shutdown():

            pid_cmd_vel_msg.linear.x = self.pid_linear()
            pid_cmd_vel_msg.angular_z= self.pid_angular()

            cmd_vel_repub.publish(pid_cmd_vel_msg)
            r.sleep()

    def pid_linear(self):
        """ do pid control here """

        # linear PID
        self.error_linear= sqrt((self.goal_x-self.odom_x)**2-(self.goal_y-self.odom_y)**2)#desired position - current position, always positive
        self.P_value_linear=self.linear_kp*self.error_linear 
        self.D_value_linear=self.linear_kd*(self.error_linear - self.Derivator_linear) #always negative before overshoot
        self.Derivator_linear=self.error_linear
        self.Integrator_linear=self.Integrator_linear +self.error_linear

        if self.Integrator_linear > self.Integrator_max_linear:
        	self.Integrator_linear=self.Integrator_max_linear
        elif self.Integrator_linear < self.Integrator_min_linear:
        	self.Integrator_linear=self.Integrator_min_linear

        self.I_value_linear=self.Integrator_linear*self.linear_ki

        #only do compensation if inside circular region
        if self.error_linear<self.linear_threshold:
        	pid_linear_x= self.P_value_linear + self.I_value_linear + self.D_value_linear
        
        else:
        	pid_linear_x=0.0        


        
        new_linear_x=self.linear_x+pid_linear_x
        
        if new_linear_x>self.linear_velocity_threshold:
        	new_linear_x=self.linear_velocity_threshold
        elif new_linear_x<-self.linear_velocity_threshold:
        	new_linear_x=-self.linear_velocity_threshold


        return (new_linear_x)


    def pid_angular(self):
    	#angular PID
        self.error_angular= self.goal_yaw-self.odom_yaw#desired angle - current angle
        self.P_value_angular=self.angular_kp*self.error_angular
        self.D_value_angular=self.angular_kd*(self.error_angular - self.Derivator_angular)
        self.Derivator_angular=self.error_angular
        self.Integrator_angular=self.Integrator_angular +self.error_angular

        if self.Integrator_angular > self.Integrator_max_angular:
        	self.Integrator_angular=self.Integrator_max_angular
        elif self.Integrator_angular < self.Integrator_min_angular:
        	self.Integrator_angular=self.Integrator_min_angular

        self.I_value_angular=self.Integrator_angular*self.angular_ki

        #only do compensation if position is achieved
        if self.error_linear<self.angular_threshold:  
        	pid_angular_z= self.P_value_angular + self.I_value_angular + self.D_value_angular
        else:
        	pid_angular_z=0.0

        new_angular_z=self.angular_z+pid_angular_z

        if new_angular_z>self.angular_velocity_threshold:
        	new_angular_z=self.angular_velocity_threshold
        elif new_angular_z<-self.angular_velocity_threshold:
        	new_angular_z=-self.angular_velocity_threshold

        return (new_angular_z)


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
        self.angular_z = msg.angular.z  # rotation
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
