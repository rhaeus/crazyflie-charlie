#!/usr/bin/env python
from __future__ import print_function
import sys
import json 
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from numpy.core.defchararray import array
from path_planning import pathPlanning
from occupancy_grid import OccupancyGrid 

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cf_msgs.srv import DronePath, DronePathRequest, DronePathResponse 

def extract_path(req):
    path_to_file = '/home/ramona/dd2419_ws/src/maps/ramona.world.json'
    gridSize = float(10.0)
    OG = OccupancyGrid(path_to_file, gridSize = gridSize)

    OG.readWorld()
    OG.setGrid()
    OG.drawWalls()
    OG.drawGate()

    start, goal = req.start, req.goal
    start = (int(start.pose.position.x * gridSize - OG.xLim[0]), int(start.pose.position.y *gridSize - OG.yLim[0]))
    goal = (int(goal.pose.position.x * gridSize - OG.xLim[0]), int(goal.pose.position.y * gridSize- OG.yLim[0]))

    planning = pathPlanning(OG.grid)
    planning.Astar(np.array(start), np.array(goal))
    planning.extractPath()
    arr = planning.path

    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = 'map'
    path.header.seq = 0
    for i in range(len(arr)):
        temp = PoseStamped()
        temp.header.stamp = rospy.Time.now()
        temp.header.frame_id = 'map'
        temp.header.seq = i
        x = arr[i][0]/float(gridSize) - OG.xLim[1]/float(gridSize)
        temp.pose.position.x = x
        y = arr[i][1]/float(gridSize) - OG.yLim[1]/float(gridSize)
        temp.pose.position.y = y
        temp.pose.position.z = 0.4
        temp.pose.orientation.x = 0
        temp.pose.orientation.y = 0
        temp.pose.orientation.z = 0
        temp.pose.orientation.w = 1

        path.poses.append(temp)

        
    return DronePathResponse(path)

def add_two_ints_server():
    rospy.init_node('a_star_planning')
    s = rospy.Service('drone_path', DronePath, extract_path)
    print("Ready to plan.")
    rospy.spin()


if __name__ == "__main__":
    add_two_ints_server()

