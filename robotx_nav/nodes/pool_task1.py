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
from move_base_forward import Forward
from move_base_force_cancel import ForceCancel
from task1_toplevel import WaypointPublisher



def loiter(target, radius, polygon, is_ccw, is_relative):
    if target is not None: 
        loiter_obj = Loiter(nodename="loitering", target=target, radius=radius, polygon=polygon, is_ccw=is_ccw, is_relative=is_relative)

def constant_heading(goal):
    if goal is not None:
        constant_obj = Forward(nodename="constant_heading", target=goal, waypoint_separation=5, is_relative=False)


def cancel_goal():
    """ asynchronously cancel goals"""
    force_cancel = ForceCancel(nodename="forcecancel", repetition=repetition)

if __name__ == '__main__':
    # create data queue
    pool = mp.Pool(5)
    
    task1_planner=WaypointPublisher()

    pool.close()
    pool.join()