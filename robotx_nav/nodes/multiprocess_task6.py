#! /usr/bin/env python
#find the break

import time
import multiprocessing as mp
import sys
from visualization_msgs.msg import MarkerArray, Marker
from move_base_waypoint_geo import MoveToGeo
from move_base_forward import Forward
from move_base_loiter import Loiter
from move_base_zigzag import Zigzag
from move_base_force_cancel import ForceCancel
from geometry_msgs.msg import Pose, Point

markers_array=MarkerArray()
totems_pose=list()


def motion_planner():

    #while zigzag around to look for white
    #while loiter around white, 3-10m diameter, look for underwater break or another white totem
    #do constant heading starting from first white towards the direction of break or another white totem
    #if white totem available, correct heading wrt to white totem (alternative correction wrt to breaks)
    #while constant heading, run calculate breaks node






def zigzag_worker(q):

    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    zigzag_obj = Zigzag(nodename="zigzag", )
    # get the waypoints, loop wait for updates
    while not q.empty():
        loiter_obj.respawn(q.get())
    print p.name, p.pid, 'Exiting'

    

def constant_heading_worker(q):
    """ constant heading to pass the gate,
    need roi_target_identifier to give/update waypoint """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    constant_heading_obj = Forward(nodename="constant_heading", target=None, waypoint_separation=5, is_relative=False)
    # get the waypoints, loop wait for updates
    while not rospy.is_shutdown():
        if not q.empty():
            constant_heading_obj.respawn(q.get())
    print p.name, p.pid, 'Exiting'


def loiter_worker(q):

    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    loiter_obj = Loiter(nodename="loiter", )
    # get the waypoints, loop wait for updates
    while not q.empty():
        loiter_obj.respawn(q.get())
    print p.name, p.pid, 'Exiting'


def cancel_goal_worker(repetition):
    """ asynchronously cancel goals"""
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    force_cancel = ForceCancel(nodename="forcecancel", repetition=repetition)
    print p.name, p.pid, 'Exiting'



if __name__ == '__main__':

    
    # create data queue
    q = mp.Queue()
    # create workers
    zigzag_mp = mp.Process(name="zzg", target=zigzag_worker, args=(q,))
    constant_heading_mp = mp.Process(name="csh", target=constant_heading_worker, args=(q,))
    loiter_mp = mp.Process(name="ltr", target=loiter_worker, args=(q,))

    zigzag_mp.start()
    zigzag_mp.join()

    constant_heading_mp.start()
    constant_heading_mp.join()

    loiter_mp.start()
    loiter_mp.join()

    motion_planner()

    rospy.spin()

