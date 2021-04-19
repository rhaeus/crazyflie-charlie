#!/usr/bin/env python
from __future__ import print_function
import math
import numpy as np
import random
import json

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from cf_msgs.srv import ExploreReqGoal, ExploreReqGoalResponse, ExplorePoint, ExplorePointRequest, ExplorePointResponse
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_matrix, translation_matrix, rotation_matrix, concatenate_matrices, quaternion_about_axis

from grid_map import GridMap
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

global map_file_path
global map_resolution
global inflation_radius
global explore_radius
global grid_map
global safe_spots
global safe_spot_offset
global safe_spot_index
safe_spot_index = 0
global state # 0=safe spot, 1=random position
state = 0

def explore_req_goal(req):
    # alternate between random position (out of 10) with highest score
    # and safe spot
    # TODO maybe choose closest safe spot

    global map_file_path
    global map_resolution
    global inflation_radius
    global explore_radius
    global grid_map
    global safe_spots
    global safe_spot_offset
    global safe_spot_index
    global state

    is_safe_zone = False

    goal = PoseStamped()
    goal.header.frame_id = 'map'

    if state == 0: # use safe spot
        goal.pose.position.x = safe_spots[safe_spot_index][0]
        goal.pose.position.y = safe_spots[safe_spot_index][1]
        goal.pose.position.z = safe_spots[safe_spot_index][2]
        goal.pose.orientation.x = safe_spots[safe_spot_index][3][0]
        goal.pose.orientation.y = safe_spots[safe_spot_index][3][1]
        goal.pose.orientation.z = safe_spots[safe_spot_index][3][2]
        goal.pose.orientation.w = safe_spots[safe_spot_index][3][3]

        safe_spot_index += 1
        if safe_spot_index >= len(safe_spots):
            safe_spot_index = 0
        state = 1
        is_safe_zone = True

    elif state == 1: # random position
        # sample 10 random points in map space and pick the one with highest score for new explored space
        pose = (0,0, -1) # x, y, score
        for i in range(0, 10):
            x = random.uniform(grid_map.b_min[0], grid_map.b_max[0])
            y = random.uniform(grid_map.b_min[1], grid_map.b_max[1])
            score = explore_circle((x,y), explore_radius, grid_map.explored_space, 0)
            if score > pose[2]:
                pose = (x,y,score)
    
        goal.pose.position.x = pose[0]
        goal.pose.position.y = pose[1]
        goal.pose.position.z = 0.4
        (goal.pose.orientation.x,
        goal.pose.orientation.y,
        goal.pose.orientation.z,
        goal.pose.orientation.w) = quaternion_from_euler(0,0,0)

        is_safe_zone = False

        state = 0

    safe = Bool()
    safe.data = is_safe_zone

        
    return ExploreReqGoalResponse(goal, safe)

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

def get_safe_spots():
    global safe_spots
    safe_spots = []
    global map_file_path
    global safe_spot_offset

    with open(map_file_path, 'rb') as f:
        world = json.load(f)

    mark = []
    sign = []
    for m in world['markers']:
        if m["id"]==1:
            continue
        mark.append(m)
    for s in world['roadsigns']:
        sign.append(s)

    objects = [item for sublist in zip(mark,sign) for item in sublist]
    objects.reverse()

    for o in objects:
        translation = (o['pose']['position'][0], o['pose']['position'][1], o['pose']['position'][2])
        roll, pitch, yaw = o['pose']['orientation']
        q = quaternion_from_euler(math.radians(roll),
                                            math.radians(pitch),
                                            math.radians(yaw))
                        
        # get rotation and translation matrices of object
        rot = quaternion_matrix(q)
        t = translation_matrix(translation)
        m = concatenate_matrices(t, rot)

        # we want to be <safe_spot_offset> in front of the object
        # the objecct coordinate system has the y axis coming out of the object 
        # plane, so we want an offset in the y direction
        offset = np.array([0, safe_spot_offset, 0, 1])

        # we transform the offset to object space
        # that is our goal position
        pos = np.dot(m, offset)

        # get z axis transformed to object space
        z = np.array([0, 0, 1, 1])
        z_trans = np.dot(rot, z)
        z_axis = (z_trans[0], z_trans[1], z_trans[2])

        # x = np.array([1, 0, 0, 1])
        # x_trans = np.dot(rot, x)
        # x_axis = (x_trans[0], x_trans[1], x_trans[2])

        # get y axis transformed to object space
        y = np.array([0, 1, 0, 1])
        y_trans = np.dot(rot, y)
        y_axis = (y_trans[0], y_trans[1], y_trans[2])

        # calculate orientation of safe spot pose so that it 
        # goes towards the object and roll and pitch are zero
        # done by first rotate to object orientation (q)
        # then rotate -90 degree around transformed z axis (q2)
        # and then rotate 90 degree around transformed y axis (q3)
        q2 = quaternion_about_axis(math.radians(-90), z_axis)
        q3 = quaternion_about_axis(math.radians(90), y_axis)
        q_inter = quaternion_multiply(q3, q2)
        new_q = quaternion_multiply(q_inter, q)

        # force roll and pitch to be 0
        # needed for when object is on floor
        # this method would output some roll and pitch that the
        # drone faces the floor = not good
        _, _, yaw2 = euler_from_quaternion(new_q)
        new_q = quaternion_from_euler(0, 0, yaw2)
        # print("roll2, pitch2, yaw2: ",math.degrees(roll2), math.degrees(pitch2), math.degrees(yaw2) )
        safe_spots.append((pos[0], pos[1], pos[2], new_q))



def explorer_server():
    rospy.init_node('explorer_service')

    global map_file_path
    global map_resolution
    global inflation_radius
    global explore_radius
    global grid_map
    global safe_spots
    global safe_spot_offset


    map_file_path = rospy.get_param('~map_file_path')
    map_resolution = rospy.get_param('~map_resolution', 0.1)
    inflation_radius = rospy.get_param('~inflation_radius', 0.1)
    explore_radius = rospy.get_param('~explore_radius', 0.4)
    safe_spot_offset = rospy.get_param('~safe_spot_offset', 0.4)

    rospy.Service('explorer_request_goal', ExploreReqGoal, explore_req_goal)
    rospy.Service('explorer_explore_point', ExplorePoint, explore_point)

    pub = rospy.Publisher('/cf1/grid_map', OccupancyGrid, queue_size=10)
    poses_pub = rospy.Publisher('/cf1/safe_spots', PoseArray, queue_size=10)

    grid_map = GridMap(map_file_path, map_resolution, inflation_radius) 
    get_safe_spots()
    pose_array = PoseArray()
    pose_array.header.frame_id = 'map'
    pose_array.header.stamp = rospy.Time.now()

    for p in safe_spots:
        pose = Pose()
        # print("p:", p)
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.position.z = p[2]
        pose.orientation.x = p[3][0]
        pose.orientation.y = p[3][1]
        pose.orientation.z = p[3][2]
        pose.orientation.w = p[3][3]
        pose_array.poses.append(pose)


    rate = rospy.Rate(10) # 10hz

    print("[explorer_service] Ready to explore")

    while not rospy.is_shutdown():
        msg = grid_map.get_ros_message()
        pub.publish(msg)
        poses_pub.publish(pose_array)
        # pub2.publish(msg2)
        rate.sleep()

    # rospy.spin()


if __name__ == "__main__":
    explorer_server()

