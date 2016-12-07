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


def stationkeeping_worker(data_q, res_q):
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    stationkeep_obj = StationKeeping(nodename="stationkeep", target=None, radius=2, duration=30)
    while True:
        target, radius, duration = data_q.get()
        print "from planner", target
        if target[2] < -1e6: # unless send a -inf z by waypoint pub: terminating
            break
        else:
            stationkeep_obj.respawn(target, radius, duration)
            res_q.put(False)  # do not onhold stationkeep
    print p.name, p.pid, 'Exiting'


def moveto_worker(data_q, res_q):
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    # get the waypoints, loop wait for updates
    moveto_obj = MoveTo("moveto", is_newnode=True, target=None, is_relative=False)
    while True:
        target = data_q.get()
        print target
        if target[2] < -1e6:  # unless send a -inf z by waypoint pub: terminating
            break
        else:
            moveto_obj.respawn(target)
            res_q.put(False)  # do not onhold moveto
    print p.name, p.pid, 'Exiting'


def planner_worker(sk_data_q, moveto_data_q, sk_res_q, moveto_res_q):
    """ plan for totems """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    planner_obj = ScanTheCode("scanthecode")
    while True:
        if not moveto_res_q.empty(): # get update from moveto on success
            hol = moveto_res_q.get()  # free moveto to be on hold after moveto finished
            planner_obj.update_hold_moveto(hol)
        if not sk_res_q.empty(): # get update from sk on success
            hol = sk_res_q.get()  # free sk to be on hold after sk finished
            planner_obj.update_hold_stationkeep(hol)

        is_led_valid, is_totem_found, sk_target, moveto_target = planner_obj.planner()  # get data
        if is_led_valid:  # finish job
            print "mp find the led"
            poison_pill = [0, 0, -float("inf")]
            sk_data_q.put([poison_pill, None, None])
            # need an exit target
            if moveto_target != []:
                print "mp", moveto_target
                moveto_data_q.put(moveto_target)
            # finally kill moveto
            time.sleep(10)
            moveto_data_q.put(poison_pill)
            time.sleep(10)
            break
        else:  # led is not valid
            if sk_target != [] and moveto_target == []:  # planner want sk_target
                print "mp stationkeep called"
                sk_data_q.put([sk_target, 2, 30])
            elif sk_target == [] and moveto_target != []:  # planner want random walk
                print "mp random walk called"
                moveto_data_q.put(moveto_target)

        time.sleep(0.5)
    print p.name, p.pid, 'Exiting'


if __name__ == "__main__":
    moveto_data_q = mp.Queue()
    moveto_res_q = mp.Queue()
    sk_data_q = mp.Queue()
    sk_res_q = mp.Queue()

    stationkeeping_mp = mp.Process(name="skp", target=stationkeeping_worker, args=(sk_data_q, sk_res_q,))
    moveto_mp = mp.Process(name="mvt", target=moveto_worker, args=(moveto_data_q, moveto_res_q,))
    planner_mp = mp.Process(name="pln", target=planner_worker, args=(sk_data_q, moveto_data_q, sk_res_q, moveto_res_q,))

    stationkeeping_mp.start()
    moveto_mp.start()
    planner_mp.start()
    # close
    stationkeeping_mp.join()
    moveto_mp.join()
    planner_mp.join()

