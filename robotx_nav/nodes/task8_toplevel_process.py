#!/usr/bin/env python

""" task 8:
    -----------------
    Created by Ren Ye @ 2016-11-06
    Authors: Ren Ye, Reinaldo
    -----------------
    find the pinger
"""

import rospy
import multiprocessing as mp
import math
import time
import numpy as np
from move_base_waypoint import MoveTo
from move_base_loiter import Loiter
from pinger_planner import Pinger

def loiter_worker(data_q, res_q):
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    loiter_obj = Loiter(nodename="loiter", target=None, polygon=6, radius=5, is_ccw=True)
    while True:
        target, polygon, radius, is_ccw = data_q.get()
        print "from planner", target
        if target[2] < -1e6: # unless send a -inf z by waypoint pub: terminating
            break
        else:
            loiter_obj.respawn(target, polygon, radius, is_ccw)
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


def planner_worker(loiter_data_q, moveto_data_q, loiter_res_q, moveto_res_q):
    """ plan for totems """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    planner_obj = Pinger("pinger")
    while True:
        if not moveto_res_q.empty(): # get update from moveto on success
            hol = moveto_res_q.get()  # free moveto to be on hold after moveto finished
            planner_obj.update_hold_moveto(hol)
        if not loiter_res_q.empty(): # get update from loiter on success
            hol = loiter_res_q.get()  # free loiter to be on hold after loiter finished
            planner_obj.update_hold_loiter(hol)

        is_exited, is_gate_find, loiter_target, moveto_target = planner_obj.planner()  # get data
        if is_exited:  # finish job
            print "mp exited from the gate"
            poison_pill = [0, 0, -float("inf")]
            loiter_data_q.put([poison_pill, None, None])
            # need an exit target
            if moveto_target != []:
                print "mp", moveto_target
                moveto_data_q.put(moveto_target)
            # finally kill moveto
            time.sleep(10)
            moveto_data_q.put(poison_pill)
            time.sleep(10)
            break
        else:  # have not finish the job
            if loiter_target != [] and moveto_target == []:  # planner want loiter_target
                print "mp loiter called"
                loiter_data_q.put(loiter_target)
            elif loiter_target == [] and moveto_target != []:  # planner want random walk
                print "mp random walk called"
                moveto_data_q.put(moveto_target)

        time.sleep(0.5)
    print p.name, p.pid, 'Exiting'


if __name__ == "__main__":
    moveto_data_q = mp.Queue()
    moveto_res_q = mp.Queue()
    loiter_data_q = mp.Queue()
    loiter_res_q = mp.Queue()

    loiter_mp = mp.Process(name="loiterp", target=loiter_worker, args=(loiter_data_q, loiter_res_q,))
    moveto_mp = mp.Process(name="mvt", target=moveto_worker, args=(moveto_data_q, moveto_res_q,))
    planner_mp = mp.Process(name="pln", target=planner_worker, args=(loiter_data_q, moveto_data_q, loiter_res_q, moveto_res_q,))

    loiter_mp.start()
    moveto_mp.start()
    planner_mp.start()
    # close
    loiter_mp.join()
    moveto_mp.join()
    planner_mp.join()

