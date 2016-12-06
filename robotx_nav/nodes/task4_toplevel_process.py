#!/usr/bin/env python

""" task 4:
    -----------------
    Created by Ren Ye @ 2016-11-06
    Authors: Ren Ye, Reinaldo
    -----------------
    scan the code
"""

import rospy
import multiprocessing as mp
import math
import time
import numpy as np
from move_base_forward import Forward
from move_base_waypoint import MoveTo
from move_base_loiter import Loiter
from move_base_stationkeeping import StationKeeping
from move_base_force_cancel import ForceCancel
from scan_the_code_planner import ScanTheCode


def stationkeeping_worker(q):
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    stationkeep_obj = StationKeeping(nodename="stationkeep", target=None, radius=2, duration=200)
    while True:
        target, radius, duration = q.get()
        print "from planner", target
        if target[2] < -1e6: # unless send a -inf z by waypoint pub: terminating
            break
        else:
            stationkeep_obj.respawn(target, radius, duration)
    print p.name, p.pid, 'Exiting'


def moveto_worker(q, hold_move_q):
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    # get the waypoints, loop wait for updates
    moveto_obj = MoveTo("moveto", is_newnode=True, target=None, is_relative=False)
    while True:
        target = q.get()
        print target
        if target[2] < -1e6: # unless send a -inf z by waypoint pub: terminating
            break
        else:
            moveto_obj.respawn(target)
            hold_move_q.put(False)
    print p.name, p.pid, 'Exiting'


def planner_worker(station_keep_q, moveto_q, hold_moveto_q):
    """ plan for totems """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    planner_obj = ScanTheCode("scanthecode")
    while True:
        if not hold_moveto_q.empty(): # get update from moveto on success
            hol = hold_move_q.get() # free moveto to be on hold after moveto finished
            planner_obj.update_hold_moveto(hol)
        isready, stationkeep_target, moveto_target, hold_moveto, completed = planner_obj.planner()  # try to find totem lamp
        if completed:  # code get
            poison_pill = [0, 0, -float("inf")]
            station_keep_q.put([poison_pill, None, None])
            # need an exit target
            if moveto_target != []:
                moveto_q.put(moveto_target)

            # finally kill moveto
            time.sleep(1)
            moveto_q.put(poison_pill)
            break
        elif isready and not completed and stationkeep_target != []:  # still have pending loiter points
            print "stationkeep called"
            station_keep_q.put(stationkeep_target)
        elif not isready and not hold_moveto and not completed and moveto_target != []:  # need to explore for valid loiter points
            moveto_q.put(moveto_target)

    print p.name, p.pid, 'Exiting'


if __name__ == "__main__":
    moveto_q = mp.Queue()
    hold_moveto_q = mp.Queue()
    station_keep_q = mp.Queue(1)
    v_dict_q = mp.Queue(1)

    stationkeeping_mp = mp.Process(name="skp", target=stationkeeping_worker, args=(station_keep_q,))
    moveto_mp = mp.Process(name="mvt", target=moveto_worker, args=(moveto_q, hold_moveto_q,))
    planner_mp = mp.Process(name="pln", target=planner_worker, args=(station_keep_q, moveto_q, hold_moveto_q,))

    stationkeeping_mp.start()
    moveto_mp.start()
    planner_mp.start()
    # close
    stationkeeping_mp.join()
    moveto_mp.join()
    planner_mp.join()

