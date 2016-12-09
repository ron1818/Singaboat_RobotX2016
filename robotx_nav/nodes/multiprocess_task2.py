#! /usr/bin/env python

""" Mission 2-FIND TOTEMS AND AVOID OBSTACLES

    Go to totems of interest in sequence
	move to an offset distance from totem position
	do loiter according to direction
	remove totem from list

    After all totems visited (list of target empty) or duration time out
	terminate mission

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

def movetso_worker(q):
    """ go to a particular waypoint,
    normally very near to a totem """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    moveto_obj = MoveTo(nodename="moveto", target=None, is_relative=False)
    # get the waypoints, loop wait for updates

    while  True:
        target = q.get()
        if target[2] < -1e6:
            break
        else:
            moveto_obj.respawn(target)
    print p.name, p.pid, 'Exiting'

def loitedfdr_worker(q):
    """ roi give target, according to color, do loitering """
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    loiter_obj = MoveTo(nodename="loiter", target=None, is_relative=False)
    while  True:
        target, is_ccw = q.get()
        if target[2] < -1e6:
            break
        else:
            waypoint_obj.respawn(target, radius=None, is_ccw=is_ccw)
    print p.name, p.pid, 'Exiting'
    pass

def waypoint_publisher_worker(conn, moveto_q, loiter_q):
    """ spawn waypoints for both moveto and loter,
    for moveto, only need [x, y, heading],
    for loiter, need [[x, y, heading], is_ccw] """
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
    q.put([-5, 15, -1.57])
    time.sleep(5)
    q.put([-10, 15, 1.57])
    time.sleep(5)
    q.put([0, 0, 0])
    q.put([0, 0, -float('inf')])
    conn.send('exit') # exit
    # waypoint_publisher_obj = Waypoint_Publisher()
    # # push the waypoints
    # while not waypoint_publisher_obj.complete():
    queue, connect = xxx.pub_waypoint()
    q.put(queue)
    conn.send(connect)
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
    manager = mp.Manager()
    moveto_q = manager.Queue()
    loiter_q = manager.Queue()
    parent_conn, child_conn = mp.Pipe(duplex=True)
    pool = mp.Pool(processes=4)
    # initialize objects
    # potential problem: rospy_init_node can only be called once in a py code
    loiter_worker = Loiter(nodename="loiter")
    print "a"
    moveto_worker = MoveToGeo(nodename="moveto", is_newnode=False)
    print "b"
    moveto_worker.respawn([1.344, 103.456, 0])
    # move to a random point to collect some data, hopefully can get some colored totem
    # pool.apply_async(moveto_worker, args=(target,), callback=None)
    # if we have a valid totem, loiter around it
    # pool.apply_async(loiter_worker, args=(target, is_ccw), callback=None)
    # else, keep move to some random point from different region

    # # release
    # # create workers
    # loiter_mp = mp.Process(name="loiter", target=loiter_worker, args=(loiter_q,))
    # moveto_mp = mp.Process(name="mvt", target=moveto_worker, args=(moveto_q,))
    # cancel_goal_mp = mp.Process(name="ccg", target=cancel_goal_worker, args=(child_conn, 5,))
    # waypoint_publisher_mp = mp.Process(name="wpt", target=waypoint_publisher_worker, args=(parent_conn, waypoint_queue,))
    # # can run at the background
    # roi_mp.daemon = True

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

