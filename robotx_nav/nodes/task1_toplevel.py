#!/usr/bin/env python

""" task 1:
    -----------------
    Created by Ren Ye @ 2016-11-06
    Authors: Ren Ye, Reinaldo
    -----------------

    <put the descriptions from robotx.org pdf file>

    <put the algorithms in natural language, can use bullet points, best is to use markdown format>

    <if you have plan b, can put it here>

    ## example ##
    + Go to start point
    + Rotate in position to detect red_1 and green_1 buoys
    + Plot perpendicular waypoints wrt to position of red and green buoys
    + Move towards waypoints move_base_forward
    + meanwhile requesting positions of red_2 and green_2
    + 	shutdown move_base_forward, create new move_base_forward towards mid of red_2 and green_2

    <change log put here>
    ### @ 2016-11-06 ###
    + create template

    renye's approach:
    1. drive to gps waypoint
    2. slowly in place rotate # noneed
    3. detect red and green totems by any camera
    4. rotate to bow to red and green totems
    5. roi of red in bow/left and roi of green in bow/right, calculate center
    6. drive until roi vanishes from both bow cameras, detect totem from port and starboard
    7. see new roi from bow
    8. drive with 5 and 6


"""

import rospy
import numpy as np
import multiprocessing as mp
import sys
import math
from geometry_msgs.msg import Point, Pose
from move_base_util import MoveBaseUtil
from move_base_waypoint_geo import MoveToGeo
from move_base_force_cancel import ForceCancel
from move_base_forward import Forward
from visualization_msgs.msg import MarkerArray, Marker

class WaypointPublisher(object):
    x0, y0, yaw0= 0, 0, 0

    markers_array=MarkerArray()
    red_totem=list()
    green_totem=list()

    def __init__(self):

        rospy.init_node('task_1', anonymous=True)
        rospy.Subscriber("/marker", MarkerArray, self.marker_callback, queue_size = 50)
        self.marker_pub= rospy.Publisher('waypoint_markers', Marker, queue_size=5)

        self.odom_received = False
        # rospy.wait_for_message("/odom", Odometry)
        # rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=50)
        rospy.wait_for_message("/odometry/filtered/global", Odometry)
        rospy.Subscriber("/odometry/filtered/global", Odometry, self.odom_callback, queue_size=50)
        while not self.odom_received:
            rospy.sleep(1)



    def pub_waypoint():
        distance=15
        if self.search_marker(3, 0, self.red_totem) and self.search_marker(3, 1, self.green_totem):
            
            #find closest available totem pairs
            for i in range(len(self.red_totem)):
                if i==0:
                    [x_red, y_red]=[self.red_totem[i].pose.position.x, self.red_totem[i].pose.position.y]
                    d_red=self.distance_from_boat(x_red, y_red) 
                elif i>0 and self.distance_from_boat(self.red_totem[i].pose.position.x, self.red_totem[i].pose.position.y)<d_red:
                    [x_red, y_red]=[self.red_totem[i].pose.position.x, self.red_totem[i].pose.position.y]
                    d_red=self.distance_from_boat(x_red, y_red) 

            for i in range(len(self.green_totem)):
                if i==0:
                    [x_green, y_green]=[self.green_totem[i].pose.position.x, self.green_totem[i].pose.position.y]
                    d_green=self.distance_from_boat(x_green, y_green) 
                elif i>0 and self.distance_from_boat(self.green_totem[i].pose.position.x, self.green_totem[i].pose.position.y)<d:
                    [x_green, y_green]=[self.green_totem[i].pose.position.x, self.green_totem[i].pose.position.y]
                    d_green=self.distance_from_boat(x_green, y_green) 

            if math.sqrt((x_red-x_green)**2+(y_red-y_green)**2) <20:
                [x_center, y_center]=([x_red, y_red]+[x_green, y_green])/2
                theta=math.atan2(y_red-y_green, x_red-x_green)-math.pi/2
            else:
                [x_center, y_center]=([x_red, y_red]+[x_green, y_green])/2
                theta=math.atan2(y_red-y_green, x_red-x_red)+math.atan2(10/30)

            return [x_center+distance*math.cos(theta), y_center+distance*math.sin(theta), theta]
        else:
            pass

    def distance_from_boat(x, y):
        return math.sqrt((x-self.x0)**2+(y-self.y0)**2)

    def is_complete():

    
    def search_marker(self, obj_type, obj_color, pose_list)
        #pose_list is list that stores Pose obj for requested markers
        if markers_array.markers.size()>0:
            for i in range(markers_array.markers.size()):
                if markers_array.markers[i].type == obj_type and markers_array.markers[i].id==obj_color:
                    #may append more than 1 markers
                    pose_list.append(markers_array.markers[i].Pose)
            return True
        else:
            return False



    def marker_callback(self, msg):
        #updates markers_array
        self.markers_array=msg 
        #visualize markers in rviz
        for i in range(msg->markers.size()):
            self.marker_pub.publish(msg->markers)


    def odom_callback(self, msg):
        """ call back to subscribe, get odometry data:
        pose and orientation of the current boat,
        suffix 0 is for origin """
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _, _, self.yaw0 = euler_from_quaternion((x, y, z, w))
        self.odom_received = True
    

if __name__ == '__main__':
    try:
        WaypointPublisher()
        # stage 1: gps
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
