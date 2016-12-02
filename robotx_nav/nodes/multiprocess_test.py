#! /usr/bin/env python

import time
import multiprocessing as mp
import sys
from move_base_waypoint_geo import MoveToGeo
from move_base_force_cancel import ForceCancel

def gps_waypoint(q, nodename, target_geos):
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    gps_waypoint = MoveToGeo(nodename="movetogeo_test", target=target_geos[0])
    for target_geo in target_geos:
        gps_waypoint.respawn(target_geo)
        time.sleep(2)
        print "next point"
        q.put("nextpoint")
    print p.name, p.pid, 'Exiting'

def cancel_goal(q, nodename, repetition):
    p = mp.current_process()
    print p.name, p.pid, 'Starting'
    counter = 0
    while counter <= 20:
        counter += 1
        time.sleep(1)
        q.put(0)
    else:
        force_cancel = ForceCancel(nodename="forcecancel", repetition=10)
        q.put(1)
    print p.name, p.pid, 'Exiting'


if __name__ == '__main__':
    q = mp.Queue()
    gps = mp.Process(name="gps", target=gps_waypoint, args=(q, "gps_test",[(1.344423, 103.684952, 0), (1.344469, 103.684666, 0)]))
    cancel = mp.Process(name="cancel", target=cancel_goal, args=(q, "cancel_test", 10))

    gps.start()
    cancel.start()
    print (q.get())
    cancel.join()
    gps.join()
