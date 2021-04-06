#!/usr/bin/env python
from __future__ import print_function
import math
import numpy as np
import random

import rospy
from geometry_msgs.msg import PoseStamped
from cf_msgs.srv import Explore, ExploreResponse
from tf.transformations import quaternion_from_euler

from grid_map import GridMap

global map_file_path
global map_resolution
global inflation_radius

def explore(dummy):
    # for now just return random position in map
    # TODO implement smarter strategy

    global map_file_path
    global map_resolution
    global inflation_radius

    grid_map = GridMap(map_file_path, map_resolution, inflation_radius) 

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = random.uniform(grid_map.b_min[0], grid_map.b_max[0])
    goal.pose.position.y = random.uniform(grid_map.b_min[1], grid_map.b_max[1])
    # goal.pose.position.x = 1.5
    # goal.pose.position.y = 0.5
    goal.pose.position.z = 0.4
    (goal.pose.orientation.x,
    goal.pose.orientation.y,
    goal.pose.orientation.z,
    goal.pose.orientation.w) = quaternion_from_euler(0,0,0)

        
    return ExploreResponse(goal)

def explorer_server():
    rospy.init_node('explorer_service')

    global map_file_path
    global map_resolution
    global inflation_radius

    map_file_path = rospy.get_param('~map_file_path')
    map_resolution = rospy.get_param('~map_resolution', 0.1)
    inflation_radius = rospy.get_param('~inflation_radius', 0.1)
    rospy.Service('explorer', Explore, explore)
    print("Ready to explore")
    rospy.spin()


if __name__ == "__main__":
    explorer_server()

