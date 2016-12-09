#!/usr/bin/env python
# ren ye 20160517
# refernence:
# http://answers.ros.org/question/29706/twist-message-example-and-cmd_vel/
# http://blog.csdn.net/heyijia0327/article/details/41823809

# changelog, use arduino + rosserial to drive the esc

# import roslib
import rospy
import math
# import tf.transformations
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension  # for PWM signal
from dynamic_reconfigure.server import Server
from robotx_control.cfg import CalibrateMotorConfig
# from auv_ros_control.PCA9685PW import PCA9685PW
# from auv_ros_control.SC16IS750_I2C import SC16IS750

""" use ROS to send cmd_vel to AUV boat with
1) differential propeller drive
2) one propeller and one rudder
3) vector setup with 4 propellers
"""


class Differential(object):
    """ two propellers identical mounted at bow,
    speed controlled by PWM via Pi's I2C and PCA9685"""
    _WHEEL_W = 0.3

    def __init__(self, mode=1, i2c_address=0x40, pwm_channels=[0, 1]):

        rospy.init_node('auv_driver', anonymous=True)
        self.mode = rospy.get_param("~mode", mode)
        if self.mode == 0:  # i2c mode
            self.pwm = PCA9685PW(address=i2c_address, debug=True)
            self.pwm.initial_esc(pwm_channels)
            self.pwm_channels = pwm_channels
        else:  # rosserial mode
            self.pwm_pub = rospy.Publisher("pwm", UInt16MultiArray, queue_size=10)


    def constrain(self, x):
        """ constain the cmd vel value to max speed """
        if x <= (-1 * self.max_speed):
            return -1 * self.max_speed
        elif x >= self.max_speed:
            return self.max_speed
        else:
            return x

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

    def callback(self, msg):
        """ callback the subscribe, get Twist data
        and output to PWM signal """

        # rospy.loginfo("Received a /cmd_vel message!")
        # rospy.loginfo("L: [%f, %f, %f], A:[%f, %f, %f]"
        #               % (msg.linear.x, msg.linear.y, msg.linear.z,
        #                  msg.angular.x, msg.angular.y, msg.angular.z))

        cmd_twist_rotation = msg.angular.z  # rotation
        cmd_twist_x = msg.linear.x
        cmd_twist_y = msg.linear.y

        # Twist -> differential motor speed
        propellerspeed = self.odom_to_speed(cmd_twist_x, cmd_twist_y, cmd_twist_rotation)

        # correct the speed
        corrected_speed = self.correct_speed(propellerspeed)
        # rospy.loginfo(corrected_speed)
        # send to the pwm chip
        if self.mode == 0:
            self.i2c_send(corrected_speed)
        else:
            self.arduino_send(corrected_speed)

    def correct_speed(self, propellerspeed):
        """ correct by scaling, offset and constrain """
        # correct polarity
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

    def i2c_send(self, speed):
        """ output differential signals to pca9685pw
        with max speed and zero speed, check PCA9685 for details """
        pwm = self.speed_to_pwm(speed)
        for i in range(len(speed)):
            print pwm[i] / 1000.0
            self.pwm.setServoPulse(self.pwm_channels[i], pwm[i] / 1000.0)

    def arduino_send(self, speed):
        pwm_msg = UInt16MultiArray()
        # pwm_msg.layout.dim = MultiArrayDimension()
        # pwm_msg.layout.dim.label = "pwm_value"
        # pwm_msg.layout.dim.size = len(self.pwm_channels)
        pwm_msg.data = self.speed_to_pwm(speed)
        # rospy.loginfo(pwm_msg.data)

        self.pwm_pub.publish(pwm_msg)

    def speed_to_pwm(self, speed):
        # by experiment 20161130 nanyang lake
        min_pwm = 1200
        neutral_pwm = 1500
        max_pwm = 1750
        # pwm = k * speed + b
        k = (max_pwm - min_pwm) / (self.max_speed + self.max_speed)
        b = neutral_pwm
        return [int(k * s + b) for s in speed]

    def odom_to_speed(self, cmd_twist_x=0, cmd_twist_y=0, cmd_twist_rotation=0):

        cent_speed = cmd_twist_x  # center speed (m/s)
        # differential speed
        yawrate2 = self.yawrate_to_speed(cmd_twist_rotation)

        if self.steering_direction:
            Lwheelspeed = cent_speed + yawrate2 / 2
            Rwheelspeed = cent_speed - yawrate2 / 2
        else:
            Lwheelspeed = cent_speed - yawrate2 / 2
            Rwheelspeed = cent_speed + yawrate2 / 2

        return Lwheelspeed, Rwheelspeed

    def yawrate_to_speed(self, yawrate):
        """ yawrate=dt*(RWs-LWs)/d """
        return yawrate * self._WHEEL_W / self._D_T

    def listener(self):
        rospy.Subscriber("/cmd_vel", Twist, self.callback)  # /cmd_vel
        self.srv = Server(CalibrateMotorConfig, self.dynamic_callback)
        rospy.spin()


class Vector4(Differential):
    """4 thrusters in a vector form front two have outer 45 as +ve,
    rear two have inner 45 as +ve:
        /(PWM1) \(PWM2)
        \(PWM3) /(PWM4)
    """

    def __init__(self, mode=0, i2c_address=0x40, pwm_channels=[0, 1, 2, 3]):
        super(Differential, self).__init__(mode, i2c_address, pwm_channels)

    def odom_to_speed(self, cmd_twist_x=0, cmd_twist_y=0,
                      cmd_twist_rotation=0):
        twist_mag = math.sqrt(cmd_twist_x ** 2 + cmd_twist_y ** 2)
        twist_ang = math.cos(cmd_twist_x / twist_mag)
        twist_14 = twist_ang / math.cos(twist_ang - math.pi / 4)
        twist_23 = twist_ang / math.sin(twist_ang - math.pi / 4)

        # differential speed
        yawrate2 = self.yawrate_to_speed(cmd_twist_rotation)

        if self.steering_direction:
            twist_1 = twist_14 - yawrate2 / 4
            twist_4 = twist_14 + yawrate2 / 4
            twist_2 = twist_23 - yawrate2 / 4
            twist_3 = twist_23 + yawrate2 / 4
        else:
            twist_1 = twist_14 + yawrate2 / 4
            twist_4 = twist_14 - yawrate2 / 4
            twist_2 = twist_23 + yawrate2 / 4
            twist_3 = twist_23 - yawrate2 / 4

        return twist_1, twist_2, twist_3, twist_4

    def correct_speed(self, propellerspeed):
        """ correct by scaling, offset and constrain """
        # correct polarity
        if self.left_polarity:
            corrected_twist_1 = propellerspeed[0] * \
                self.left_scale_factor + self.left_offset
            corrected_twist_3 = propellerspeed[2] * \
                self.left_scale_factor + self.left_offset
        else:
            corrected_twist_1 = -1 * propellerspeed[0] * \
                self.left_scale_factor + self.left_offset
            corrected_twist_3 = -1 * propellerspeed[2] * \
                self.left_scale_factor + self.left_offset

        if self.right_polarity:
            corrected_twist_2 = propellerspeed[1] * \
                self.right_scale_factor + self.right_offset
            corrected_twist_4 = propellerspeed[3] * \
                self.right_scale_factor + self.right_offset
        else:
            corrected_twist_2 = -1 * propellerspeed[1] * \
                self.right_scale_factor + self.right_offset
            corrected_twist_4 = -1 * propellerspeed[3] * \
                self.right_scale_factor + self.right_offset

        # constrain
        return (self.constrain(corrected_twist_1),
                self.constrain(corrected_twist_2),
                self.constrain(corrected_twist_3),
                self.constrain(corrected_twist_4))


if __name__ == '__main__':
    try:
        auv_control = Differential(mode=1)
        # auv_control = Vector4()
        auv_control.listener()
    except rospy.ROSInterruptException:
        pass
