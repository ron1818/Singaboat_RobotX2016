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
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension  # for PWM signal
from dynamic_reconfigure.server import Server
from robotx_control.cfg import CalibrateMotorConfig
# from robotx_control.PCA9685PW import PCA9685PW
# from robotx_control.SC16IS750_I2C import SC16IS750

""" use ROS to send cmd_vel to AUV boat with
1) differential propeller drive
2) one propeller and one rudder
3) vector setup with 4 propellers
"""


class Differential(object):
    """ two propellers identical mounted at bow,
    speed controlled by PWM via (mode 0) Pi's I2C and PCA9685
    or via (mode 1) rosserial-arduino """
    _WHEEL_W = 2.1

    def __init__(self):
        rospy.init_node('start_motor', anonymous=True)
        # self.max_speed = rospy.get_param("~max_speed", 1.3)
        # pwm publisher initialize
        self.pwm_pub = rospy.Publisher("pwm", UInt16MultiArray, queue_size=10)

    def dynamic_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: \
        {left_throttle_scale}, {right_throttle_scale}, \
        {left_throttle_offset}, {right_throttle_offset}, \
        {max_speed}, {yaw_rate}, {steering_direction}, \
        {left_throttle_polarity}, {right_throttle_polarity}, \
        """.format(**config))

        self.left_scale_factor = config["left_throttle_scale"]
        self.right_scale_factor = config["right_throttle_scale"]
        self.left_offset = config["left_throttle_offset"]
        self.right_offset = config["right_throttle_offset"]
        self.max_speed = config["max_speed"]
        self._D_T = config["yaw_rate"]
        self.steering_direction = config["steering_direction"]
        self.left_polarity = config["left_throttle_polarity"]
        self.right_polarity = config["right_throttle_polarity"]
        return config

    def constrain(self, x):
        """ for constrain the motor speed """
        if x <= (-1 * self.max_speed):
            return -1 * self.max_speed
        elif x >= self.max_speed:
            return self.max_speed
        else:
            return x

    def callback(self, msg):
        """ callback the subscribe, get Twist data
        and output to PWM signal """

        # rospy.loginfo("Received a /cmd_vel message!")
        # rospy.loginfo("L: [%f, %f, %f], A:[%f, %f, %f]"
        #               % (msg.linear.x, msg.linear.y, msg.linear.z,
        #                  msg.angular.x, msg.angular.y, msg.angular.z))

        cmd_twist_rotation = msg.angular.z  # rotation
        cmd_twist_x = msg.linear.x
        # cmd_twist_y = msg.linear.y  # 0 by default

        # Twist -> differential motor speed
        propellerspeed = self.odom_to_speed(cmd_twist_x, cmd_twist_rotation)

        # correct the speed
        corrected_speed = self.correct_speed(propellerspeed)
        rospy.loginfo(corrected_speed)
        # send to the pwm chip
        self.arduino_send(corrected_speed)

    def correct_speed(self, propellerspeed):
        """ correct by scaling, offset and constrain """
        # correct polarity
        # scale then offset
        if self.left_polarity:
            corrected_left_speed = propellerspeed[0] * \
                self.left_scale_factor + self.left_offset
        else:
            corrected_left_speed = -1 * propellerspeed[0] * \
                self.left_scale_factor + self.left_offset

        if self.right_polarity:
            corrected_right_speed = propellerspeed[1] * \
                self.right_scale_factor + self.right_offset
        else:
            corrected_right_speed = -1 * propellerspeed[1] * \
                self.right_scale_factor + self.right_offset

        # constrain
        return (self.constrain(corrected_left_speed),
                self.constrain(corrected_right_speed))

    # def i2c_send(self, speed):
    #     """ output differential signals to pca9685pw
    #     with max speed and zero speed, check PCA9685 for details """
    #     for i in range(len(speed)):
    #         self.pwm.setThrottle(self.pwm_channels[i], speed[i])

    def arduino_send(self, speed):
        """ output differential signals to rosserial arduino """
        pwm_msg = UInt16MultiArray()
        # pwm_msg.layout.dim = MultiArrayDimension()
        # pwm_msg.layout.dim.label = "pwm_value"
        # pwm_msg.layout.dim.size = len(self.pwm_channels)
        pwm_msg.data = self.speed_to_pwm(speed)
        rospy.loginfo(pwm_msg.data)

        self.pwm_pub.publish(pwm_msg)

    def speed_to_pwm(self, speed):
        min_pwm = 1100
        neutral_pwm = 1500
        max_pwm = 1900
        # pwm = k * speed + b
        k = (max_pwm - min_pwm) / (self.max_speed + self.max_speed)
        b = neutral_pwm
        return [int(k * s + b) for s in speed]

    def odom_to_speed(self, cmd_twist_x=0, cmd_twist_rotation=0):

        cent_speed = cmd_twist_x  # center speed (m/s)
        # differential speed
        yawrate2 = self.yawrate_to_speed(cmd_twist_rotation)

        # correct steering direction
        if self.steering_direction:
            Lwheelspeed = cent_speed - yawrate2 / 2
            Rwheelspeed = cent_speed + yawrate2 / 2
        else:
            Lwheelspeed = cent_speed + yawrate2 / 2
            Rwheelspeed = cent_speed - yawrate2 / 2

        return Lwheelspeed, Rwheelspeed

    def yawrate_to_speed(self, yawrate):
        """ yawrate=dt*(RWs-LWs)/d """
        return yawrate * self._WHEEL_W / self._D_T

    def listener(self):
        # subscribe to cmd vel and dynamic callback
        rospy.Subscriber("cmd_vel", Twist, self.callback)  # cmd_vel
        self.srv = Server(CalibrateMotorConfig, self.dynamic_callback)
        rospy.spin()


if __name__ == '__main__':
    try:
        auv_control = Differential()
        auv_control.listener()
    except rospy.ROSInterruptException:
        pass
