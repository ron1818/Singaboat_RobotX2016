#!/usr/bin/env python

""" Mission 3-IDENTIFY SYMBOLS AND DOCK

    Set map border, size and origin
    Do zigzag scouting to look for dock/dock's face
    Move toward dock until certain distance
    Identify each symbols of interest, may rotate around and stay. Better if lines of symbol board can be extracted to get normal of board
    Go to direction of symbols in sequence
	plot waypoints towards dock. Stop in dock for some time, Reverse blindly

    After all bays are visited, terminate mission and return

"""

import time
import multiprocessing as mp
import sys
from move_base_waypoint_geo import MoveToGeo
from move_base_forward import Forward
from move_base_loiter import Loiter
from object_follower import ObjectFollower #TODO
from move_base_stationkeeping import StationKeeping
from move_base_reverse import Reverse
from move_base_force_cancel import ForceCancel
from roi_waypoint import RoiWaypoint #TODO
from roi_rotate import RoiRotate #TODO

# def gps_worker(target):
#     """ go to gps point """
#     p = mp.current_process()
#     print p.name, p.pid, 'Starting'
#     gps_obj = MoveToGeo(nodename="gps_waypoint", target=None)
#     # spawn the gps coordinate, one time only
#     gps_obj.respawn(target)
#     print p.name, p.pid, 'Exiting'

# def constant_heading_worker(q):
#     """ constant heading to pass the gate,
#     need roi_target_identifier to give/update waypoint """
#     p = mp.current_process()
#     print p.name, p.pid, 'Starting'
#     constant_heading_obj = Forward(nodename="constant_heading", target=None, waypoint_separation=5, is_relative=False)
#     # get the waypoints, loop wait for updates
#     while not q.empty():
#         constant_heading_obj.respawn(q.get())
#     print p.name, p.pid, 'Exiting'

def loiter_worker(q):
    """ roi give target, according to color, do loitering """
    pass

def object_follower_worker(pipe): #TODO
    """ follow an identified roi until minimum distance is met """
    pass

def reverse_worker(q, distance): #TODO
    """ after one successful docking, reverse to a distance/duration """
    pass

def roi_target_identifier_worker(q): #TODO
    """ spawn way points based on roi of the cameras, will end by itself """
    pass  #TODO

def roi_rotate_worker():
    """ rotate based on the four camera's readings, turn the bow to the totems """
    pass #TODO

def cancel_goal_worker(repetition):
    """ asynchronously cancel goals"""
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    force_cancel = ForceCancel(nodename="forcecancel", repetition=repetition)
    print p.name, p.pid, 'Exiting'


if __name__ == '__main__':
    # create data queue
    q = mp.Queue()
    pipe = mp.Pipe()
    # create workers
    # gps_target = [1.344423, 103.684952, 1.57]
    # gps_mp = mp.Process(name="gps", target=gps_worker, args=(gps_target,))
    # loiter_mp = mp.Process(name="loiter", target=loiter_worker, args=(q,))
    # constant_heading_mp = mp.Process(name="csh", target=constant_heading_worker, args=(q,))
    roi_rotate_mp = mp.Process(name="roi", target=roi_rotate_worker, args=(,))
    roi_target_mp = mp.Process(name="roi", target=roi_target_identifier_worker, args=(q,))


    # # start to locate waypoints, need filtering
    # roi_mp.start()
    # # stage1: go to gps points
    # gps_mp.start()
    # gps_mp.join()
    # # stage2: in place turning to make the bow camera facing the totems
    # roi_rotate_mp.start()
    # roi_rotate_mp.join()
    # # stage3: start constant heading
    # constant_heading_mp.start()
    # constant_heading_mp.join()

