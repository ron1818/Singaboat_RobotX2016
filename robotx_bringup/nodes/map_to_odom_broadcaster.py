#!/usr/bin/env python
import roslib
import rospy
import tf
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix
from geographiclib.geodesic import Geodesic
from math import pi, cos, sin

class MapOdomTF():
    lat0, lon0 = 0, 0
    def __init__(self, nodename, map_lat, map_lon, rotation):
        rospy.init_node(nodename)
        map_lat = rospy.get_param("~map_lat", map_lat)
        map_lon = rospy.get_param("~map_lon", map_lon)
        rotation = rospy.get_param("~rotation", rotation)
        rate = rospy.Rate(10)
        br = tf.TransformBroadcaster()
        rospy.wait_for_message("navsat/fix", NavSatFix)
        rospy.Subscriber("navsat/fix", NavSatFix, self.navsat_fix_callback, queue_size=10)
        rospy.sleep(10)
        print map_lat, map_lon
        print self.lat0, self.lon0
        initial_gps = [self.lat0, self.lon0]

        result = Geodesic.WGS84.Inverse(map_lat, map_lon, self.lat0, self.lon0)
        r = result['s12']
        azi = result['azi1'] * pi /180.0
        theta = pi / 2 - azi

        xyz = [r * cos(theta), r * sin(theta), 0]
        print xyz
        rpy = [0, 0, rotation]

        while not rospy.is_shutdown():
            self.tf_link(br, "map", "odom", xyz, rpy)
            rate.sleep()


    def navsat_fix_callback(self, msg):
        self.lat0 = msg.latitude
        self.lon0 = msg.longitude



    def tf_link(self, br, parent_frame, child_frame, xyz=(0,0,0), rpy=(0,0,0)):
        r, p, y = rpy
        br.sendTransform(xyz,
                         tf.transformations.quaternion_from_euler(r, p, y),
                         rospy.Time.now(),
                         child_frame,
                         parent_frame)

if __name__ == '__main__':
    MapOdomTF("map_odom_br", 1.344446, 103.684945, -135/180.0*pi)
