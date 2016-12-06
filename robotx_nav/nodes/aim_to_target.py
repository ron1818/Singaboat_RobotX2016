#!/usr/bin/env python
# Reinaldo 5-12-16
# Class to aim to a target by rotational pid controller

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from robotx_nav.cfg import AimConfig


class Aim(object):
    
    angular_kp, angular_ki, angular_kd =  1, 1, 1

    odom_x, odom_y, odom_yaw = 0, 0, 0

    angular_velocity_threshold=0.3

    terminate=False

    def __init__(self, target, duration):

        rospy.init_node('aim', anonymous=True)
        r = rospy.Rate(10)

        rospy.Subscriber("odometry/filtered/global", Odometry, callback=self.odom_callback, queue_size=10)
 
        # rospy.spin()
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

        while not rospy.is_shutdown() and not (rospy.get_time() - start_time) < duration and not self.terminate:
        
            pid_cmd_vel_msg.angular.z= self.pid_angular(target)

            cmd_vel_pub.publish(pid_cmd_vel_msg)
            r.sleep()




    def pid_angular(self, target):
    	#angular PID
        angle_error=math.atan2(target[1]-self.odom_y, target[0]-self.odom_x)-self.odom_yaw

        self.error_angular=math.atan2(math.sin(angle_error), math.cos(angle_error)) #trick to remap to -pi -
        self.P_value_angular=self.angular_kp*self.error_angular

        derivative_error=self.error_angular - self.Derivator_angular
        self.D_value_angular=self.angular_kd*math.atan2(math.sin(derivative_error), math.cos(derivative_error))
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


    def odom_callback(self, msg):
        """ callback the subscribe, get pose data """
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _, _, self.odom_yaw = euler_from_quaternion((x, y, z, w))

    def shutdown(self):
        self.terminate=True



if __name__ == '__main__':
    try:
        aim_to_target = Aim(target=[0, 0], 30)

    except KeyboardInterrupt, rospy.ROSInterruptException:
        pass
