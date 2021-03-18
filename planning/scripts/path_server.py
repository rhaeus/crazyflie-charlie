#!/usr/bin/env python
from __future__ import print_function
import sys
import json 
import math
import numpy as np
 

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cf_msgs.srv import DronePath, DronePathRequest, DronePathResponse 

from grid_map import GridMap
from a_star import AStar


global map_file_path
global map_resolution
global inflation_radius

def plan_path(req):
    global map_file_path
    global map_resolution
    global inflation_radius

    grid_map = GridMap(map_file_path, map_resolution, inflation_radius) 

    start, goal = req.start, req.goal

    astar = AStar(grid_map)

    path_indices = astar.plan(start, goal)
    # print("path", path_indices)


    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = 'map'
    path.header.seq = 0
    for i in range(len(path_indices)):
        temp = PoseStamped()
        temp.header.stamp = rospy.Time.now()
        temp.header.frame_id = 'map'
        temp.header.seq = i
        (x, y) = grid_map.grid_index_to_coord((path_indices[i]))
        temp.pose.position.x = x
        temp.pose.position.y = y
        temp.pose.position.z = 0.4
        temp.pose.orientation.x = 0
        temp.pose.orientation.y = 0
        temp.pose.orientation.z = 0
        temp.pose.orientation.w = 1

        path.poses.append(temp)

        
    return DronePathResponse(path)

def path_server():
    rospy.init_node('path_server')

    global map_file_path
    global map_resolution
    global inflation_radius

    map_file_path = rospy.get_param('~map_file_path')
    map_resolution = rospy.get_param('~map_resolution', 0.1)
    inflation_radius = rospy.get_param('~inflation_radius', 0.1)

    s = rospy.Service('drone_path', DronePath, plan_path)
    print("Ready to plan path with A*.")
    rospy.spin()



if __name__ == "__main__":
    path_server()

