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
from geometry_msgs.msg import Point
from move_base_util import MoveBaseUtil
from move_base_waypoint_geo import MoveToGeo
from move_base_force_cancel import ForceCancel
from move_base_forward import Forward
from roi_coordinate import RoiCoordinate
import matplotlib.pyplot as plt

class Task1(object):
    bow_left_red_x = float("Inf")
    bow_right_red_x = float("Inf")
    port_red_x = float("Inf")
    starboard_red_x = float("Inf")
    transom_red_x = float("Inf")
    bow_left_green_x = float("Inf")
    bow_right_green_x = float("Inf")
    port_green_x = float("Inf")
    starboard_green_x = float("Inf")
    transom_green_x = float("Inf")
    red_x_list = list()
    red_y_list = list()
    green_x_list = list()
    green_y_list = list()

    def subscribe_coordinate(self, topic_name, colorname):
        rospy.Subscriber(topic_name, Point, self.roi_coordinate_callback, colorname, queue_size=10)

    def __init__(self):
        # 0. parameters
        self.target_lat = rospy.get_param("~lat", 1.344470)
        self.target_lon = rospy.get_param("~lon", 103.684937)
        self.target_heading = rospy.get_param("~heading", 0)  # north

        rospy.init_node("task1")
        rospy.on_shutdown(self.shutdown)
        # rate = rospy.Rate(10)
        self.start_time = rospy.get_time()
        self.duration = 2

        q = mp.Queue()

        # initialize objects
        gps_target = [(self.target_lat, self.target_lon, self.target_heading)]
        self.gps_waypoint = MoveToGeo(nodename="gps", target=None, is_newnode=False)
        self.constant_heading = Forward(nodename="constant_heading", target=None, waypoint_separation=5, is_relative=True, is_newnode=False)
        # three nodes into worker
        # 1. drive to gps waypoint
        gps_move_to_mp = mp.Process(name="gps", target=self.gps_worker,
                                 args=(gps_target))
        # 2. roi to get target point, daemon
        # roi_get_target_mp = mp.Process(name="roi_target", target=self.roi_worker,
        #                         args=(q,))
        # roi_get_target.daemon = True

        # 3. constant heading, must get updated target
        constant_heading_mp = mp.Process(name="constant_heading", target=self.constant_heading_worker,
                                      args=(q,))

        # 4. cancel goal worker
        cancel_goal_mp = mp.Process(name="cancel_goal", target=self.cancel_goal_worker,
                                 args=("cancel_goal", 10))


        # roi detect color totem, assume already have in launch
        # then listen to namespace/objectname/colorname/coordinates

        # must launch first:

        self.subscribe_coordinate("bow/left/totem/red/coordinate", "red")
        self.subscribe_coordinate("bow/right/totem/red/coordinate", "red")
        # self.subscribe_coordinate("port/totem/redcoordinate", "red")
        # self.subscribe_coordinate("starboard/totem/redcoordinate", "red")
        # self.subscribe_coordinate("transom/totem/redcoordinate", "red")
        self.subscribe_coordinate("bow/left/totem/green/coordinate", "green")
        self.subscribe_coordinate("bow/right/totem/green/coordinate", "green")
        # self.subscribe_coordinate("port/totem/greencoordinate", "green")
        # self.subscribe_coordinate("starboard/totem/greencoordinate", "green")
        # self.subscribe_coordinate("transom/totem/greencoordinate", "green")
        # rospy.wait_for_message("bow/left/totem/red/coordinate", Point)

        q.put([10,0,0])
        # gps_move_to_mp.start()
        # roi_get_target_mp.start()
        # gps_move_to_mp.join()
        # wait for self.roi_target to be valid
        # use queue?
        constant_heading_mp.start()
        constant_heading_mp.join()
        # roi_get_target_mp.join()

    def roi_coordinate_callback(self, msg, colorname):
        if rospy.get_time() - self.start_time < self.duration:
            if colorname == "red":
                if msg.x is not None and msg.y is not None and msg.x < 10000 and msg.y < 10000:
                    self.red_x_list.extend([msg.x])
                    self.red_y_list.extend([msg.y])
            elif colorname == "green":
                if msg.x is not None and msg.y is not None and msg.x < 10000 and msg.y < 10000:
                    self.green_x_list.extend([msg.x])
                    self.green_y_list.extend([msg.y])
            self.is_ready = False
        else:
            self.is_ready = True
            self.start_time = rospy.get_time()

    def shutdown(self):
        pass

    def gps_worker(self, target_geo):
        p = mp.current_process()
        print p.name, p.pid, 'Starting'
        self.gps_waypoint.respawn(target_geo)
        print p.name, p.pid, 'Exiting'

    def roi_worker(self, q):
        p = mp.current_process()
        print p.name, p.pid, 'Starting'
        # calculate the center point bewtween red and green totem
        # the simplest way: median for (x_red, y_red) and median for (x_green, y_green)
        # then take the centerpoint for constant heading
        # the hard way: collect red and green points and use svm to get the separation plane
        # use the plane for constant heading
        self.is_ready = False
        red_x_center, red_y_center = 0, 0
        green_x_center, green_y_center = 0, 0
        plt.ion()
        fig, ax = plt.subplots()
        plot = ax.scatter([], [])
        ax.set_xlim(-30, 30)
        ax.set_ylim(0, 70)
        while not rospy.is_shutdown() or self.roi_target is None:
            try:
                # print len(self.red_x_list), len(self.red_y_list)
                # print len(self.green_x_list), len(self.green_y_list)
                ax.scatter(self.red_x_list, self.red_y_list, color="r")
                ax.scatter(self.green_x_list, self.green_y_list, color="g")
                # plt.show()
            except:
                pass

            if self.is_ready:
                red_x_center, red_y_center = np.median(self.red_x_list), np.median(self.red_y_list)
                green_x_center, green_y_center = np.median(self.green_x_list), np.median(self.green_y_list)
                roi_target = [(red_x_center + green_x_center) / 2.0, (red_y_center + green_y_center) / 2.0, 0]
                q.put(roi_target)
                rate.sleep()
            fig.canvas.draw()
        else:
            pass
        print p.name, p.pid, 'Exiting'


    def constant_heading_worker(self, q):
        p = mp.current_process()
        print p.name, p.pid, 'Starting'
        ####
        if not q.empty():
            target = q.get()
            print target
            self.constant_heading.respawn(target)
        print p.name, p.pid, 'Exiting'

    def cancel_goal_worker(self, nodename, repetition):
        p = mp.current_process()
        print p.name, p.pid, 'Starting'
        counter = 0
        while counter <= 20:
            counter += 1
            time.sleep(1)
        else:
            force_cancel = ForceCancel(nodename=nodename, repetition=10)
        print p.name, p.pid, 'Exiting'



if __name__ == '__main__':
    try:
        Task1()
        # stage 1: gps
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
