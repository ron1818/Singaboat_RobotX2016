#!/usr/bin/env python
import rospy
from sensor_msgs.msg import RegionOfInterest, Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
import numpy
import tf
from math import pi, cos, sin
from move_base_util import MoveBaseUtil
import time

class RoiCoordinate(object):
    """ take roi and do calculations """
    x_offset, y_offset, height, width = 0, 0, 0, 0
    x_offset_list = list()
    y_offset_list = list()
    height_list = list()
    width_list = list()
    coordinate = [float('Inf'), float('Inf')]

    def __init__(self, nodename, namespace="bow/left", objectname="totem", colorname="red",
                 camera_frame="camera_link", base_frame=None, fixed_frame=None, rov=1.41):
        rospy.init_node(nodename)
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)

        self.namespace = rospy.get_param('~namespace', namespace)
        self.objectname = rospy.get_param('~objectname', objectname)
        self.colorname= rospy.get_param('~colorname', colorname)
        self.camera_frame = rospy.get_param('~camera_frame', camera_frame)
        self.base_frame = rospy.get_param('~base_frame', base_frame)
        self.fixed_frame = rospy.get_param('~fixed_frame', fixed_frame)

        # append namespace and camera frame
        self.camera_frame = self.namespace + "/" + self.camera_frame

        self.rov = rospy.get_param('~rov', 1.41)
        self.totem_width = rospy.get_param('~totem_width', 0.25)
        self.accumulate = rospy.get_param('~accumulate', 10)  # 30 images
        self.image_width = 0
        self.image_height = 0
        self.roi = RegionOfInterest()
        self.is_pub_coordinate = False
        self.image_count = 0

        self.tf_listener = tf.TransformListener()
        pub_node = self.namespace + "/" + self.objectname + "/" + self.colorname + "/coordinate"
        self.coordinate_pub = rospy.Publisher(pub_node, Point, queue_size = 5)
        self.coordinate_msg = Point()

        rospy.loginfo("Waiting for camera_info topic...")

        # rospy.wait_for_message('bow/left/camera_info', CameraInfo)
        # info_node = self.namespace + "/" + self.objectname + "/" + self.colorname + "/camera_info"
        info_node = self.namespace + "/camera_info"
        rospy.wait_for_message(info_node, CameraInfo)

        # Subscribe to the camera_info topic to get the image width and height
        # rospy.Subscriber('/bow/left/camera_info', CameraInfo, self.get_camera_info, queue_size=1)
        rospy.Subscriber(info_node, CameraInfo, self.get_camera_info, queue_size=10)

        # Wait until we actually have the camera data
        while self.image_width == 0 or self.image_height == 0:
            rospy.sleep(1)

        # Wait until we have an ROI to follow
        rospy.loginfo("Waiting for an ROI to track...")

        roi_node = self.namespace + "/" + self.objectname + "/" + self.colorname + "/roi"
        # roi_node = self.namespace + "/roi"
        rospy.wait_for_message(roi_node, RegionOfInterest)
        rospy.loginfo("ROI messages detected. Starting localization...")
        self.roi_subscriber = rospy.Subscriber(roi_node, RegionOfInterest, self.roi_callback, queue_size=10)
        while not rospy.is_shutdown():
            # print self.is_pub_coordinate
            if self.is_pub_coordinate:
                coordinate = self.calculate_coordinate(self.x_offset, self.width,
                                                     self.image_width, self.rov,
                                                     self.camera_frame, self.fixed_frame)
                self.coordinate_msg.x = coordinate[0]
                self.coordinate_msg.y = coordinate[1]
                # print self.coordinate_msg
                self.coordinate_pub.publish(self.coordinate_msg)
            # else:
            #     self.coordinate_pub.publish(Point(float("Inf"), float("Inf"), 0))
            r.sleep()


    def roi_callback(self, msg):
        """ calculate average over accumulate """
        if self.image_count <= self.accumulate:
            self.x_offset_list.append(msg.x_offset)
            self.y_offset_list.append(msg.y_offset)
            self.height_list.append(msg.height)
            self.width_list.append(msg.width)
            self.is_pub_coordinate = False
            self.image_count += 1
        else:
            self.x_offset = numpy.median(self.x_offset_list)
            self.y_offset = numpy.median(self.y_offset_list)
            self.height = numpy.median(self.height_list)
            self.width = numpy.median(self.width_list)
            self.x_offset_list = list()
            self.y_offset_list = list()
            self.height_list = list()
            self.width_list = list()
            self.is_pub_coordinate = True
            self.image_count = 0

    def calculate_coordinate(self, x_offset, width, image_width, rov, camera_frame, fixed_frame):
        """ use xy_offset and height width to calculate position
        with known totem size
        camera frame: +x: towards lens, +y: right hand"""
        # roi's x position used to calculate heading
        image_x_position = x_offset + width / 2.0
        # in polar form: theta: heading, r: distance
        theta = image_x_position / image_width * rov - rov / 2.0
        # unreliable: act length = radius * arc radian
        r = self.totem_width / (width / image_width * rov)
        # catersian wrt camera
        x_target_camera, y_target_camera = r * cos(theta), r * sin(theta)
        if fixed_frame is None:
            return [x_target_camera, y_target_camera]
        else:
            # tranformation from base to camera
            x_target_base, y_target_base = self.transform_tf(x_target_camera, y_target_camera,
                                                             fixed_frame, camera_frame)
            # if map_frame is None:
            #     return [x_target_base, y_target_base]
            # else:
            #     # transformation from map to base
            #     x_target_map, y_target_map = self.transform_tf(x_target_base, y_target_base,
            #                                                    map_frame, base_frame)
            #     return [x_target_map, y_target_map]
            return [x_target_base, y_target_base]

    def get_tf(self, fixed_frame, base_frame):
        """ transform from fixed frame to base frame """
        trans_received = False
        while not trans_received:
            try:
                (trans, rot) = self.tf_listener.lookupTransform(fixed_frame,
                                                                base_frame,
                                                                rospy.Time(0))
                trans_received = True
                return (Point(*trans), Quaternion(*rot))
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                pass

    def transform_tf(self, x_target_base, y_target_base, fixed_frame, base_frame):
        """ get the (x, y) wrt fixed frame from (x, y) wrt base frame"""
        # (x, y, yaw) of the base frame wrt fixed frame
        (trans, rot) = self.get_tf(fixed_frame, base_frame)
        x_base_fixed, y_base_fixed = trans.x, trans.y
        _, _, yaw_base_fixed = tf.transformations.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
        # get the point wrt fixed
        # final vector = fixed vector + rot_mat * base vector
        x_target_fixed, y_target_fixed = x_base_fixed + \
            cos(yaw_base_fixed) * x_target_base - sin(yaw_base_fixed) * y_target_base, \
                y_base_fixed + \
                    sin(yaw_base_fixed) * x_target_base + cos(yaw_base_fixed) * y_target_base

        return [x_target_fixed, y_target_fixed]

    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Unregister the subscriber
        self.roi_subscriber.unregister()
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        roi=RoiCoordinate("roi-coordinate")
    except rospy.ROSInterruptException:
        pass



