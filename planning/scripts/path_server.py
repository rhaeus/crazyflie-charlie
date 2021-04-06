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
from tf.transformations import quaternion_from_euler

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

    start_pose, goal_pose = req.start, req.goal

    start_index = grid_map.coord_to_grid_index((start_pose.pose.position.x, start_pose.pose.position.y))
    # print("start_index: ", start_index)
    goal_index = grid_map.coord_to_grid_index((goal_pose.pose.position.x, goal_pose.pose.position.y))
    # print("goal_index: ", goal_index)

    astar = AStar(grid_map)

    print("planning path...")
    # print("start: ", start_pose)
    # print("goal:", goal_pose)
    path_indices = astar.plan(start_index, goal_index)
    # print("path", path_indices)
    
    if len(path_indices) <= 1:
        print("!no path found!")
    else:
        print("path found")
        print("sparsening path...")
        path_indices = astar.sparsen_path(path_indices)
        # print("sparse path", path_indices)

    print("planning done!")

    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = 'map'
    path.header.seq = 0
    for i in range(len(path_indices)):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'map'
        p.header.seq = i
        (x, y) = grid_map.grid_index_to_coord((path_indices[i]))
        # print("waypoint: ", x,y)
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.4
        yaw = math.atan2(y, x)
        (p.pose.orientation.x,
            p.pose.orientation.y,
            p.pose.orientation.z,
            p.pose.orientation.w) = quaternion_from_euler(0, 0, yaw)
        # p.pose.orientation.x = 0
        # p.pose.orientation.y = 0
        # p.pose.orientation.z = 0
        # p.pose.orientation.w = 1

        path.poses.append(p)

        
    return DronePathResponse(path)

def path_server():
    rospy.init_node('path_server')

    global map_file_path
    global map_resolution
    global inflation_radius

    map_file_path = rospy.get_param('~map_file_path')
    map_resolution = rospy.get_param('~map_resolution', 0.1)
    inflation_radius = rospy.get_param('~inflation_radius', 0.1)

    rospy.Service('drone_path', DronePath, plan_path)
    print("Ready to plan path with A*.")
    rospy.spin()



if __name__ == "__main__":
    path_server()

