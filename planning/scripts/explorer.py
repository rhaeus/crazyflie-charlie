#!/usr/bin/env python
from __future__ import print_function
import math
import numpy as np
import random

import rospy
from geometry_msgs.msg import PoseStamped
from cf_msgs.srv import ExploreReqGoal, ExploreReqGoalResponse, ExplorePoint, ExplorePointRequest, ExplorePointResponse
from tf.transformations import quaternion_from_euler

from grid_map import GridMap
from nav_msgs.msg import OccupancyGrid

global map_file_path
global map_resolution
global inflation_radius
global explore_radius
global grid_map

def explore_req_goal(req):
    # for now just return random position in map
    # TODO implement smarter strategy

    global map_file_path
    global map_resolution
    global inflation_radius
    global explore_radius
    global grid_map

    # sample 10 random points in map space and pick the one with highest score for new explored space
    pose = (0,0, -1) # x, y, score
    for i in range(0, 10):
        x = random.uniform(grid_map.b_min[0], grid_map.b_max[0])
        y = random.uniform(grid_map.b_min[1], grid_map.b_max[1])
        score = explore_circle((x,y), explore_radius, grid_map.explored_space, 0)
        if score > pose[2]:
            pose = (x,y,score)
    
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    # goal.pose.position.x = random.uniform(grid_map.b_min[0], grid_map.b_max[0])
    # goal.pose.position.y = random.uniform(grid_map.b_min[1], grid_map.b_max[1])
    # goal.pose.position.x = 1.5
    # goal.pose.position.y = 0.5
    goal.pose.position.x = pose[0]
    goal.pose.position.y = pose[1]
    goal.pose.position.z = 0.4
    (goal.pose.orientation.x,
    goal.pose.orientation.y,
    goal.pose.orientation.z,
    goal.pose.orientation.w) = quaternion_from_euler(0,0,0)

        
    return ExploreReqGoalResponse(goal)

def explore_point(req):
    global explore_radius
    global grid_map

    point = req.point_explored
    x = point.pose.position.x
    y = point.pose.position.y
    print("explore point: ", (x,y))
    explore_circle((x,y), explore_radius, grid_map.explored_space, 1)

    return ExplorePointResponse()


def explore_circle(center_coord, radius_m, value, mode):
    # mode: 1 = fill and evaluate, 0 = only evaluate (count how much new space would be explored)
    global grid_map

    x_center, y_center = grid_map.coord_to_grid_index(center_coord)
    # get cell center of center coord for nice circle
    center_coord = grid_map.grid_index_to_coord((x_center, y_center))
    r_cells = int(radius_m / grid_map.resolution)
    bbx_min = (x_center-r_cells, y_center-r_cells)
    bbx_max = (x_center+r_cells, y_center+r_cells)
    count = 0

    # clip bbx
    bbx_min = grid_map.clip_index(bbx_min)
    bbx_max = grid_map.clip_index(bbx_max)

    for x in range(bbx_min[0], bbx_max[0] + 1):
        for y in range(bbx_min[1], bbx_max[1] + 1):
            if grid_map.get_value((x,y)) == value:
                continue
            coord = grid_map.grid_index_to_coord((x,y))
            if math.hypot(center_coord[0] - coord[0], center_coord[1] - coord[1]) <= radius_m:
                if grid_map.get_value((x,y)) == grid_map.occupied_space or grid_map.get_value((x,y)) == grid_map.c_space:
                    continue
                # cast ray to see if obstacle in the way 
                # (note: 2D for now, so even if drone could see over obstacle it will still be handled as blocked)
                ray = grid_map.raytrace((x_center, y_center), (x,y))
                ray.append((x,y))
                for cell in ray:
                    if grid_map.get_value(cell) == grid_map.occupied_space or grid_map.get_value(cell) == grid_map.c_space:
                        break

                    if grid_map.get_value(cell) == value:
                        continue

                    if mode == 1: #fill
                        grid_map.set_value(cell, value)

                    count += 1
    return count

def explorer_server():
    rospy.init_node('explorer_service')

    global map_file_path
    global map_resolution
    global inflation_radius
    global explore_radius
    global grid_map


    map_file_path = rospy.get_param('~map_file_path')
    map_resolution = rospy.get_param('~map_resolution', 0.1)
    inflation_radius = rospy.get_param('~inflation_radius', 0.1)
    explore_radius = rospy.get_param('~explore_radius', 0.4)

    rospy.Service('explorer_request_goal', ExploreReqGoal, explore_req_goal)
    rospy.Service('explorer_explore_point', ExplorePoint, explore_point)

    pub = rospy.Publisher('/cf1/grid_map', OccupancyGrid, queue_size=10)

    grid_map = GridMap(map_file_path, map_resolution, inflation_radius) 


    rate = rospy.Rate(10) # 10hz

    print("[explorer_service] Ready to explore")

    while not rospy.is_shutdown():
        msg = grid_map.get_ros_message()
        pub.publish(msg)
        # pub2.publish(msg2)
        rate.sleep()

    # rospy.spin()


if __name__ == "__main__":
    explorer_server()

