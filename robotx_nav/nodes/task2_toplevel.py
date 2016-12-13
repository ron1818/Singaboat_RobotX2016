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



def loiter_worker(res_q, data_q):
    """ go to gps point """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    loiter_obj = Loiter("loiter", is_newnode=True, target=None,
                                radius=2.5, polygon=4, is_ccw=True, is_relative=False)
    # spawn the gps coordinate, one time only
    while True:
        cid, target, radius, is_ccw = data_q.get()
        print "from planner", target
        if target[2] < -1e6: # unless send a -inf z by waypoint pub: terminating
            break
        else:
            loiter_obj.respawn(target=target, radius=radius, is_ccw=is_ccw)
            res_q.put(False)  # hold_loiter, id of assignment
    print p.name, p.pid, 'Exiting'


def moveto_worker(res_q, data_q):
    """ constant heading to pass the gate,
    need roi_target_identifier to give/update waypoint """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    # get the waypoints, loop wait for updates
    moveto_obj = MoveTo("moveto", is_newnode=True, target=None, mode=1, mode_param=1, is_relative=False)
    while True:
        target = data_q.get()
        if target[2] < -1e6: # unless send a -inf z by waypoint pub: terminating
            break
        else:
            moveto_obj.respawn(target)
            res_q.put(False)
    print p.name, p.pid, 'Exiting'
>>>>>>> ca922447663b0d1c6f5d0a4059b7a7e404984b99


# not required
# def cancel_goal_worker(conn, repetition):
#	 """ asynchronously cancel goals"""
#	 p = mp.current_process()
#	 print p.name, p.pid, 'Starting'
#	 while True:
#		 command = conn.recv()
#		 print 'child: ', command
#		 if command == 'cancel': # cancel goal
#			 print 'doing cancelling'
#			 force_cancel = ForceCancel(nodename="forcecancel", repetition=repetition)
#			 conn.send('cancelled')
#		 elif command == 'exit': # complete
#			 print "cancel goal complete, exit"
#			 break
#		 else:  # conn.recv() == 0, idle, wait for command
#			 pass
#		 time.sleep()
#
#	 print p.name, p.pid, 'Exiting'



def planner_worker(loiter_res_q, loiter_data_q, moveto_res_q, moveto_data_q):
    """ plan for totems """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    planner_obj = ColorTotemPlanner("color_planner")
    while True:
        if not loiter_res_q.empty(): # get update from loiter
            # hold_loiter, visit_id = loiter_res_q.get()
            hold_loiter = loiter_res_q.get()
            planner_obj.update_loiter(hold_loiter)  # update loiter and visit
        if not moveto_res_q.empty(): # get update from moveto on success
            planner_obj.update_hold_moveto(moveto_res_q.get())

        totem_find, loiter_target, moveto_target, allvisited = planner_obj.planner()  # try to find onhold loiter target
        # print isready
        if allvisited:  # all visited, kill all worker and exit
            poison_pill = [0, 0, -float("inf")]
            loiter_data_q.put([None, poison_pill, None, None])
            # need an exit target
            if moveto_target != []:
                moveto_data_q.put(moveto_target)

            # finally kill moveto
            time.sleep(1)
            moveto_data_q.put(poison_pill)
            break
        elif loiter_target != [] and moveto_target == []:  # still have pending loiter points
            print "loiter called"
            loiter_data_q.put(loiter_target)
        elif loiter_target == [] and moveto_target != []:  # need to moveto
            print "moveto called"
            moveto_data_q.put(moveto_target)

        time.sleep(1)


	print p.name, p.pid, 'Exiting'


if __name__ == "__main__":

    moveto_data_q = mp.Queue()
    moveto_res_q = mp.Queue()
    loiter_data_q = mp.Queue()
    loiter_res_q = mp.Queue()
    # manager = mp.Manager()
    # visited_dict = manager.dict()
    # visited_dict = {"red": False, "green": False, "blue": False, "yellow": False}

    loiter_mp = mp.Process(name="ltr", target=loiter_worker, args=(loiter_res_q, loiter_data_q,))
    moveto_mp = mp.Process(name="mvt", target=moveto_worker, args=(moveto_res_q, moveto_data_q,))
    planner_mp = mp.Process(name="pln", target=planner_worker, args=(loiter_res_q, loiter_data_q, moveto_res_q, moveto_data_q,))


	loiter_mp.start()
	moveto_mp.start()
	# cancel_goal_mp.start()
	planner_mp.start()
	# close
	loiter_mp.join()
	moveto_mp.join()
	planner_mp.join()
