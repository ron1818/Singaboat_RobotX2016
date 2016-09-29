#! /usr/bin/env python
# Ren Ye
# 2016-08-02

# import roslib
import rospy
import math
from i2c_util.sonar_uart import DST
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion  # TransformStamped
from sensor_msgs.msg import Imu, Range, Temperature
import tf


class sonar_imu(object):
    """ listen to sonar and compass data and publish to odom,
    sonar speed is the center speed x_center
    compass direction change is the theta """
    DT = 1  # DST update rate is 1 Hz
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    imu_z = 0
    v_x = 0

    def __init__(self, port, baud, imu_topic, odom_topic,
                 depth_topic, temperature_topic,
                 fixed_frame, odom_frame):
        self.imu_topic = imu_topic
        self.odom_topic = odom_topic
        self.depth_topic = depth_topic
        self.temperature_topic = temperature_topic
        self.odom_frame = odom_frame
        self.fixed_frame = fixed_frame
        self.DST = DST(port, baud)
        rospy.loginfo("port connected")

    def close(self):
        self.DST.close()
        rospy.loginfo("port closed")

    def talker(self):
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        odom_broadcaster = tf.TransformBroadcaster()
        odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        depth_pub = rospy.Publisher(self.depth_topic, Range, queue_size=10)
        temperature_pub = rospy.Publisher(self.temperature_topic,
                                          Temperature, queue_size=10)
        rospy.init_node('sonar_publisher', anonymous=True)
        odom_msg = Odometry()
        odom_quart = Quaternion()
        depth_msg = Range()
        temperature_msg = Temperature()

        odom_msg.pose.covariance = [0]*36
        odom_msg.twist.covariance = [0]*36

        r = rospy.Rate(20)

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0

        while not rospy.is_shutdown():
            # get linear velocity
            data = self.DST.next()
            # print data
            if data is not None:
                # print data
                for key, value in data.iteritems():
                    if key == "speed":
                        self.v_x = value
                        rospy.loginfo("speed is %f", self.v_x)
                    elif key == "depth":
                        self.depth = value
                        rospy.loginfo("depth is %f", self.depth)
                    elif key == "temperature":
                        self.temperature = value
                        rospy.loginfo("temperature is %f", self.temperature)
                    else:
                        rospy.loginfo("data received is invalid")
            else:
                rospy.loginfo("no data is reveived")

            # get current time
            self.current_time = rospy.Time.now()
            # change in time
            self.DTT = (self.current_time - self.last_time).to_sec()
            rospy.loginfo("time is %f", self.DTT)

            # angular displacement
            self.delta_theta = self.imu_z * self.DTT
            # linear displacement in x
            self.delta_x = (self.v_x * math.cos(self.theta)) * self.DTT
            # linear displacement in y
            self.delta_y = (self.v_x * math.sin(self.theta)) * self.DTT

            # current position
            self.x_pos += self.delta_x
            self.y_pos += self.delta_y
            self.theta += self.delta_theta

            # calculate heading in quaternion
            # rotate z upside down
            odom_quart = tf.transformations.\
                quaternion_from_euler(math.pi, 0, self.theta)
            # rospy.loginfo(odom_quart)
            odom_broadcaster.sendTransform((self.x_pos, self.y_pos, 0),
                                           odom_quart, rospy.Time.now(),
                                           self.fixed_frame, self.odom_frame)

            odom_msg.header.stamp = self.current_time
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.pose.pose.position.x = self.x_pos
            odom_msg.pose.pose.position.y = self.y_pos
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = odom_quart[0]
            odom_msg.pose.pose.orientation.y = odom_quart[1]
            odom_msg.pose.pose.orientation.z = odom_quart[2]
            odom_msg.pose.pose.orientation.w = odom_quart[3]
            odom_msg.pose.covariance[0] = 0.00001
            odom_msg.pose.covariance[7] = 0.00001
            odom_msg.pose.covariance[14] = 1000000000000.0
            odom_msg.pose.covariance[21] = 1000000000000.0
            odom_msg.pose.covariance[28] = 1000000000000.0
            odom_msg.pose.covariance[35] = 0.001

            odom_msg.child_frame_id = self.fixed_frame
            odom_msg.twist.twist.linear.x = self.v_x
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = self.imu_z
            odom_msg.twist.covariance[0] = 0.00001
            odom_msg.twist.covariance[7] = 0.00001
            odom_msg.twist.covariance[14] = 1000000000000.0
            odom_msg.twist.covariance[21] = 1000000000000.0
            odom_msg.twist.covariance[28] = 1000000000000.0
            odom_msg.twist.covariance[35] = 0.001

            depth_msg.radiation_type = Range.ULTRASOUND
            depth_msg.header.stamp = self.current_time
            depth_msg.header.frame_id = self.odom_frame
            depth_msg.field_of_view = 3 * 2 / math.pi
            depth_msg.min_range = 0.2
            depth_msg.max_range = 80
            depth_msg.range = self.depth

            temperature_msg.header.stamp = self.current_time
            temperature_msg.header.frame_id = self.odom_frame
            temperature_msg.temperature = self.temperature
            temperature_msg.variance = 0.0

            odom_pub.publish(odom_msg)
            depth_pub.publish(depth_msg)
            temperature_pub.publish(temperature_msg)

            self.last_time = self.current_time
            r.sleep()

    def listener(self):
        rospy.init_node('imu_listener')
        rospy.spin()

    def imu_callback(self, msg):
        if msg.angular_velocity.z > -0.05 and msg.angular_velocity.z < 0:
            self.imu_z = 0
        else:
            # reversed
            self.imu_z = -1 * msg.angular_velocity.z
            # rospy.loginfo("angle is %f", self.imu_z)


if __name__ == "__main__":

    port = rospy.get_param('~port', "/dev/ttyAMA0")
    baud = rospy.get_param('~baud', 4800)
    imu_topic = rospy.get_param('~imu_topic', "imu/data")
    odom_topic = rospy.get_param('~odom_topic', "sonar/odom")
    depth_topic = rospy.get_param('~depth_topic', "sonar/depth")
    temperature_topic = rospy.get_param('~temperature_topic',
                                        "sonar/temperature")
    fixed_frame = rospy.get_param('~fixed_frame', "base_footprint")
    odom_frame = rospy.get_param('~odom_frame', "odom")

    try:
        odom_publisher = sonar_imu(port, baud, imu_topic, odom_topic,
                                   depth_topic, temperature_topic,
                                   fixed_frame, odom_frame)
        odom_publisher.talker()

    except rospy.ROSInterruptException:
        odom_publisher.close()  # Close DST serial port
