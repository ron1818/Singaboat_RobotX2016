#! /usr/bin/env python
""" 1. go to gps point (optional, and may be removed in the latest competition instruction)
1.1. if no gps point, we can move by hand
2. require a waypoint publisher based on marker array to publish a waypoint
3. constant heading wait for the waypoint update, until no more waypoint is published
4. kill all process
"""

import time
import multiprocessing as mp
import sys
from move_base_waypoint_geo import MoveToGeo
from move_base_forward import Forward
from move_base_force_cancel import ForceCancel
# from roi_waypoint import RoiWaypoint #TODO
# from roi_rotate import RoiRotate #TODO

def gps_worker(target):
    """ go to gps point """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    gps_obj = MoveToGeo(nodename="gps_waypoint", target=None)
    # spawn the gps coordinate, one time only
    gps_obj.respawn(target)
    print p.name, p.pid, 'Exiting'

def constant_heading_worker(q):
    """ constant heading to pass the gate,
    need roi_target_identifier to give/update waypoint """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    constant_heading_obj = Forward(nodename="constant_heading", target=None, waypoint_separation=5, is_relative=False)
    # get the waypoints, loop wait for updates
    while True:
        target = q.get()
        print target
        if target[2] < -1e6: # unless send a -inf z by waypoint pub: terminating
            break
        else:
            constant_heading_obj.respawn(target)
    print p.name, p.pid, 'Exiting'

def waypoint_publisher_worker(conn, q):
    """ spawn way points based on marker array, will end by no more waypoint"""
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    conn.send('idle')  # ask cancel goal to be idle
    ##########fake test ###############
    q.put([0, 10, 0])
    time.sleep(5)
    conn.send('cancel') # too much, cancel it
    time.sleep(5)
    while True:
        if conn.recv() == 'cancelled':
           break
    print "resumed"
    time.sleep(10)
    conn.send('idle') # remain idle
    q.put([5, 15, -1.57])
    time.sleep(5)
    q.put([10, 15, 1.57])
    time.sleep(5)
    q.put([0, 0, 0])
    q.put([0, 0, -float('inf')])
    conn.send('exit') # exit
    # waypoint_publisher_obj = Waypoint_Publisher()
    # # push the waypoints
    # while not waypoint_publisher_obj.complete():
    #     q.put(waypoint_publisher_obj.pub_waypoint())
    #     conn.send(waypoint_publisher_obj.pub_cancel_goal())
    #     conn.send(0) # after one clear action, make it idle
    # else:
    #     # clear the queue
    #     q.task_done()
    #     conn.send(-1)  # task done
    #     # with q.mutex:
    #     #     q.queue.clear()
    print p.name, p.pid, 'Exiting'

def cancel_goal_worker(conn, repetition):
    """ asynchronously cancel goals"""
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    while True:
        command = conn.recv()
        print 'child: ', command
        if command == 'cancel': # cancel goal
            print 'doing cancelling'
            force_cancel = ForceCancel(nodename="forcecancel", repetition=repetition)
            conn.send('cancelled')
        elif command == 'exit': # complete
            print "cancel goal complete, exit"
            break
        else:  # conn.recv() == 0, idle, wait for command
            pass

    print p.name, p.pid, 'Exiting'


if __name__ == '__main__':
    # create data queue
    waypoint_queue = mp.Queue()
    parent_conn, child_conn = mp.Pipe(duplex=True)
    # create workers
    gps_target = [1.344423, 103.684952, 1.57]
    gps_mp = mp.Process(name="gps", target=gps_worker, args=(gps_target,))
    constant_heading_mp = mp.Process(name="csh", target=constant_heading_worker, args=(waypoint_queue,))
    cancel_goal_mp = mp.Process(name="ccg", target=cancel_goal_worker, args=(child_conn, 5,))
    waypoint_publisher_mp = mp.Process(name="wpt", target=waypoint_publisher_worker, args=(parent_conn, waypoint_queue,))

    # stage1: go to gps points
    # gps_mp.start()
    # gps_mp.join()
    # start to locate waypoints
    waypoint_publisher_mp.start()
    cancel_goal_mp.start()
    # stage2: start constant heading, iterative
    constant_heading_mp.start()
    constant_heading_mp.join()
    waypoint_publisher_mp.join()

    cancel_goal_mp.join()

