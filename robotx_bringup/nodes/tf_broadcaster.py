#!/usr/bin/env python
import roslib
import rospy
import tf

def tf_link(br, parent_frame, child_frame, xyz=(0,0,0), rpy=(0,0,0)):
    r, p, y = rpy
    br.sendTransform(xyz,
                     tf.transformations.quaternion_from_euler(r, p, y),
                     rospy.Time.now(),
                     child_frame,
                     parent_frame)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    base_frame = rospy.get_param('~base_frame', 'base_link')
    laser_frame = rospy.get_param('~laser_frame', 'lidar_link')
    camera_frame = rospy.get_param('~camera_frame', 'camera_link')
    gps_frame = rospy.get_param('~gps_frame', 'gps_link')
    imu_frame = rospy.get_param('~imu_frame', 'imu_link')

    rate = rospy.Rate(10)
    laser_br = tf.TransformBroadcaster()
    camera_br = tf.TransformBroadcaster()
    gps_br = tf.TransformBroadcaster()
    imu_br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        tf_link(laser_br, base_frame, laser_frame, (0,0,1.5), (0,0,0))
        tf_link(camera_br, base_frame, camera_frame, (0,0,0), (0,0,0))
        tf_link(gps_br, base_frame, gps_frame, (0,0,0), (0,0,0))
        tf_link(imu_br, base_frame, imu_frame, (0,0,0), (0,0,0))
        rate.sleep()
