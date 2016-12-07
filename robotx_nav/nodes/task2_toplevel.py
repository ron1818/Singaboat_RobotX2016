#!/usr/bin/env python
import multiprocessing as mp
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
import numpy as np
from sklearn.cluster import KMeans, DBSCAN
from sklearn import svm
from move_base_loiter import Loiter
from move_base_waypoint import MoveTo
from color_totem_planner import ColorTotemPlanner
# import tf
# from math import pi, cos, sin
# from move_base_util import MoveBaseUtil
import time


def loiter_worker(v_dict_q, q):
    """ go to gps point """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    loiter_obj = Loiter("loiter", is_newnode=True, target=None,
                                radius=2.5, polygon=4, is_ccw=True, is_relative=False)
    visited_dict = {"red": False, "green": False, "blue": False, "yellow": False}
    # spawn the gps coordinate, one time only
    while True:
        cid, target, radius, polygon, is_ccw = q.get()
        print "from planner", target
        if target[2] < -1e6: # unless send a -inf z by waypoint pub: terminating
            break
        else:
            loiter_obj.respawn(target, polygon, radius, is_ccw)
            visited_dict[cid] = True
            v_dict_q.put(visited_dict)  # dont hold moveto
    print p.name, p.pid, 'Exiting'


def moveto_worker(q, hold_move_q):
    """ constant heading to pass the gate,
    need roi_target_identifier to give/update waypoint """
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


# not required
# def cancel_goal_worker(conn, repetition):
#     """ asynchronously cancel goals"""
#     p = mp.current_process()
#     print p.name, p.pid, 'Starting'
#     while True:
#         command = conn.recv()
#         print 'child: ', command
#         if command == 'cancel': # cancel goal
#             print 'doing cancelling'
#             force_cancel = ForceCancel(nodename="forcecancel", repetition=repetition)
#             conn.send('cancelled')
#         elif command == 'exit': # complete
#             print "cancel goal complete, exit"
#             break
#         else:  # conn.recv() == 0, idle, wait for command
#             pass
#         time.sleep()
#
#     print p.name, p.pid, 'Exiting'


def planner_worker(v_dict_q, loiter_q, moveto_q, hold_move_q, cancel_conn):
    """ plan for totems """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    planner_obj = ColorTotemPlanner("color_planner")
    while True:
        if not v_dict_q.empty(): # get update from loiter on visited
            visited_dict = v_dict_q.get()
            planner_obj.update_visit(visited_dict)  # update visited
        if not hold_move_q.empty(): # get update from moveto on success
            hol = hold_move_q.get() # free moveto to be on hold after moveto finished
            planner_obj.update_hold_moveto(hol)
        isready, loiter_target, moveto_target, allvisited, hold_moveto = planner_obj.planner()  # try to find onhold loiter target
        # print isready
        if allvisited:  # all visited, kill all worker and exit
            poison_pill = [0, 0, -float("inf")]
            loiter_q.put([None, poison_pill, None, None, None])
            # need an exit target
            if moveto_target != []:
                moveto_q.put(moveto_target)

            # finally kill moveto
            time.sleep(1)
            moveto_q.put(poison_pill)
            break
        elif isready and not allvisited and loiter_target != []:  # still have pending loiter points
            print "loiter called"
            loiter_q.put(loiter_target)
        elif not isready and not hold_moveto and not allvisited and moveto_target != []:  # need to explore for valid loiter points
            print "moveto called"
            moveto_q.put(moveto_target)

    print p.name, p.pid, 'Exiting'


if __name__ == "__main__":
    moveto_q = mp.Queue()
    hold_moveto_q = mp.Queue()
    cancel_p_conn, cancel_c_conn = mp.Pipe()
    # loiter_p_conn, loiter_c_conn = mp.Pipe()
    loiter_q = mp.Queue(1)
    v_dict_q = mp.Queue(1)
    # manager = mp.Manager()
    # visited_dict = manager.dict()
    # visited_dict = {"red": False, "green": False, "blue": False, "yellow": False}

    loiter_mp = mp.Process(name="ltr", target=loiter_worker, args=(v_dict_q, loiter_q,))
    moveto_mp = mp.Process(name="mvt", target=moveto_worker, args=(moveto_q, hold_moveto_q,))
    # cancel_goal_mp = mp.Process(name="ccg", target=cancel_goal_worker, args=(cancel_p_conn, 5,))
    planner_mp = mp.Process(name="pln", target=planner_worker, args=(v_dict_q, loiter_q, moveto_q, hold_moveto_q, cancel_c_conn,))

    loiter_mp.start()
    moveto_mp.start()
    # cancel_goal_mp.start()
    planner_mp.start()
    # close
    loiter_mp.join()
    moveto_mp.join()
    planner_mp.join()
