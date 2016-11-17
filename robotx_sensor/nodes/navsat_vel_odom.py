#! /usr/bin/env python
# Ren Ye
# 2016-08-02

# import roslib
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TwistStamped
from sensor_msgs.msg import Imu
import tf
import threading


class Vel_Imu_Odom(object):
    """ listen to navsat/vel and imu/data and publish to odom,
    x_center speed is the vector sum of x and y
    compass direction change is the theta """
    DT = 1  # DST update rate is 1 Hz
    current_time = rospy.Time()
    last_time = rospy.Time()
    orientation = Quaternion()
    imu_z = 0
    vel_x = 0
    vel_y = 0

    def __init__(self, nodename):
        self.nodename = nodename
        self.vel_topic = rospy.get_param("~vel_topic", "navsat/vel")
        self.imu_topic = rospy.get_param("~imu_topic", "imu/data")
        self.odom_topic = rospy.get_param("~odom_topic", "odometry/vel")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.fixed_frame = rospy.get_param("~fixed_frame", "base_link")
        self.is_pub_tf = rospy.get_param("~is_pub_tf", False)

    def talker(self):
        rospy.Subscriber(self.vel_topic, TwistStamped, self.vel_callback)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        odom_broadcaster = tf.TransformBroadcaster()
        odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        rospy.init_node(self.nodename, anonymous=True)
        odom_msg = Odometry()
        odom_quart = Quaternion()

        odom_msg.pose.covariance = [0]*36
        odom_msg.twist.covariance = [0]*36

        r = rospy.Rate(20)

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0

        while not rospy.is_shutdown():
            # get linear velocity

            # get current time
            self.current_time = rospy.Time.now()
            # change in time
            self.DTT = (self.current_time - self.last_time).to_sec()
            # rospy.loginfo("time is %f", self.DTT)

            # angular displacement
            self.delta_theta = self.imu_z * self.DTT
            # linear displacement in x
            self.delta_x = self.vel_x * self.DTT
            # linear displacement in y
            self.delta_y = self.vel_y * self.DTT

            # current position
            self.x_pos += self.delta_x
            self.y_pos += self.delta_y
            self.theta += self.delta_theta

            # calculate heading in quaternion
            odom_quart = tf.transformations.\
                quaternion_from_euler(0, 0, self.theta)
#             rospy.loginfo(odom_quart)
            if self.is_pub_tf:
                odom_broadcaster.sendTransform((self.x_pos, self.y_pos, 0),
                                               odom_quart, rospy.Time.now(),
                                               self.fixed_frame, self.odom_frame)

            odom_msg.header.stamp = self.current_time
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.pose.pose.position.x = self.x_pos
            odom_msg.pose.pose.position.y = self.y_pos
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = self.orientation.x
            odom_msg.pose.pose.orientation.y = self.orientation.y
            odom_msg.pose.pose.orientation.z = self.orientation.z
            odom_msg.pose.pose.orientation.w = self.orientation.w
            odom_msg.pose.covariance[0] = 0.25
            odom_msg.pose.covariance[7] = 0.25
            # odom_msg.pose.covariance[0] = 0.00001
            # odom_msg.pose.covariance[7] = 0.00001
            odom_msg.pose.covariance[14] = 1000000000000.0
            odom_msg.pose.covariance[21] = 1000000000000.0
            odom_msg.pose.covariance[28] = 1000000000000.0
            # odom_msg.pose.covariance[35] = 0.001
            odom_msg.pose.covariance[35] = 0.1

            odom_msg.child_frame_id = self.fixed_frame
            odom_msg.twist.twist.linear.x = math.sqrt(self.vel_x ** 2 + self.vel_y ** 2)
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = self.imu_z
            # odom_msg.twist.covariance[0] = 0.00001
            # odom_msg.twist.covariance[7] = 0.00001
            odom_msg.twist.covariance[0] = 0.0025
            odom_msg.twist.covariance[7] = 0.00025
            odom_msg.twist.covariance[14] = 1000000000000.0
            odom_msg.twist.covariance[21] = 1000000000000.0
            odom_msg.twist.covariance[28] = 1000000000000.0
            # odom_msg.twist.covariance[35] = 0.001
            odom_msg.twist.covariance[35] = 0.1

            odom_pub.publish(odom_msg)

            self.last_time = self.current_time
            r.sleep()

    def listener(self):
        rospy.init_node('imu_listener')
        rospy.spin()

    def imu_callback(self, msg):
        self.orientation = msg.orientation
        if msg.angular_velocity.z > -0.05 and msg.angular_velocity.z < 0:
            self.imu_z = 0
        else:
            # reversed
            self.imu_z = -1 * msg.angular_velocity.z

    def vel_callback(self, msg):
            self.vel_x = msg.twist.linear.x
            self.vel_y = msg.twist.linear.y
            # rospy.loginfo(self.vel_x)


if __name__ == "__main__":

    # vel_topic = rospy.get_param('~vel_topic', "navsat/vel")
    # imu_topic = rospy.get_param('~imu_topic', "imu/data")
    # odom_topic = rospy.get_param('~odom_topic', "odometry/vel")
    # fixed_frame = rospy.get_param('~fixed_frame', "base_link")
    # odom_frame = rospy.get_param('~odom_frame', "odom")
    # is_pub_tf = rospy.get_param("~is_pub_tf", "false")

    nodename="navsat_vel_odom_node"
    try:
        odom_publisher = Vel_Imu_Odom(nodename)
        odom_publisher.talker()

    except rospy.ROSInterruptException:
        odom_publisher.close()  # Close DST serial port
