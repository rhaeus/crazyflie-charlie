#!/usr/bin/env python

import json
import sys
import math

import rospy
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Vector3, PoseArray, Pose, TransformStamped
from crazyflie_driver.msg import Position, Hover

import tf2_ros
import tf2_geometry_msgs

from cf_msgs.srv import DronePath, ExplorePoint, ExploreReqGoal
from nav_msgs.msg import Path



def localize_flag_callback(msg):
    global is_localized
    is_localized = msg.data

def trans2Map(msg):
    # marker pose is in frame camera_link
    if not tf_buf.can_transform('map', msg.header.frame_id, msg.header.stamp, rospy.Duration(1)):
        rospy.logwarn('[trans2Map] No transform from %s to map', msg.header.frame_id)
        return
    return tf_buf.transform(msg, 'map')

def publish_pos_cmd(goal):
    # Need to tell TF that the goal was just generated
    goal.header.stamp = rospy.Time.now()

    # goal is in map frame
    # drone expects goal pose in odom frame
    if not tf_buf.can_transform('cf1/odom', goal.header.frame_id, goal.header.stamp, rospy.Duration(1)):
        rospy.logwarn('[publish_pos_cmd] No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    goal_odom = tf_buf.transform(goal, 'cf1/odom')

    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = goal_odom.header.frame_id

    cmd.x = goal_odom.pose.position.x
    cmd.y = goal_odom.pose.position.y
    cmd.z = goal_odom.pose.position.z

    _, _, yaw = euler_from_quaternion((goal_odom.pose.orientation.x,
                                              goal_odom.pose.orientation.y,
                                              goal_odom.pose.orientation.z,
                                              goal_odom.pose.orientation.w))

    cmd.yaw = math.degrees(yaw)

    pub_pos_cmd.publish(cmd)

def publish_vel_cmd(goal, vx, vy, vyaw):
    # Need to tell TF that the goal was just generated
    goal.header.stamp = rospy.Time.now()

    # goal is in map frame
    # drone expects goal pose in odom frame
    if not tf_buf.can_transform('cf1/odom', goal.header.frame_id, goal.header.stamp, rospy.Duration(1)):
        rospy.logwarn('[publish_pos_cmd] No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    goal_odom = tf_buf.transform(goal, 'cf1/odom')

    cmd = Hover()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = goal_odom.header.frame_id
    cmd.vx = vx
    cmd.vy = vy
    cmd.yawrate = vyaw
    # print("vx, vy, vyaw:", vx, vy, vyaw)
    cmd.zDistance = goal_odom.pose.position.z

    pub_vel_cmd.publish(cmd)

def check_distance_to_goal(pose_map, goal_map):

    distance_to_goal = np.sqrt(math.pow(pose_map.pose.position.x - goal_map.pose.position.x,2) 
    + math.pow(pose_map.pose.position.y - goal_map.pose.position.y, 2) 
    + math.pow(pose_map.pose.position.z - goal_map.pose.position.z, 2))

    _, _, msg_yaw = euler_from_quaternion((pose_map.pose.orientation.x,
                                              pose_map.pose.orientation.y,
                                              pose_map.pose.orientation.z,
                                              pose_map.pose.orientation.w))

    _, _, goal_yaw = euler_from_quaternion((goal_map.pose.orientation.x,
                                              goal_map.pose.orientation.y,
                                              goal_map.pose.orientation.z,
                                              goal_map.pose.orientation.w))

    yaw_dist = math.atan2(math.sin(msg_yaw-goal_yaw), math.cos(msg_yaw-goal_yaw))
    yaw_dist = math.fabs(math.degrees(yaw_dist))

    return distance_to_goal, yaw_dist

def pose_callback(msg):
    global drone_pose_map
    drone_pose_map = trans2Map(msg)

def intruder_callback(msg):
    global intruder_found
    intruder_found = msg.data


rospy.init_node('brain')

global is_localized
is_localized = False

global drone_pose_map

global intruder_found
intruder_found = False

sub_flag = rospy.Subscriber("is_localized", Bool, localize_flag_callback)
pub_pos_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
pub_vel_cmd  = rospy.Publisher('/cf1/cmd_hover', Hover, queue_size=2)
sub_pose = rospy.Subscriber("/cf1/pose", PoseStamped, pose_callback)
sub_intruder = rospy.Subscriber("/cf1/intruder_detection_result", Bool, intruder_callback)
path_pub = rospy.Publisher('/path_pub', Path, queue_size=10)

tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def main(argv=sys.argv):
    global drone_pose_map
    global intruder_found
    global is_localized

    rate = rospy.Rate(10)

    state = 0
    path = []
    current_waypoint_index = -1
    current_waypoint = PoseStamped()
    goal = PoseStamped()
    is_localized = True
    done = False
    
    drone_mode = 0 # drone control mode, 0=position, 1=velocity

    while not rospy.is_shutdown():

        # publish position command if valid to prevent drone from landing
        if drone_mode == 0 and current_waypoint_index != -1:
            publish_pos_cmd(current_waypoint)

        # check if intruder is found
        if not done and intruder_found: 
            drone_mode = 0
            current_waypoint_index = 0
            current_waypoint = drone_pose_map
            state = 100
        
        if state == 0: # startup
            # liftoff
            drone_pose = rospy.wait_for_message('/cf1/pose', PoseStamped)
            start = trans2Map(drone_pose)
            if start is not None:
                start.pose.position.z = 0.4
                current_waypoint_index = 0
                current_waypoint = start
                publish_pos_cmd(current_waypoint)
                state = 5
                print("liftoff")

        if state == 5: #wait for liftoff
            publish_pos_cmd(current_waypoint)
            if abs(drone_pose_map.pose.position.z - 0.4) < 0.05:
                state = 10
                print("startup done, go to state 10")

        if state == 10: # localize
            if is_localized:
                state = 20
                print("localized, go to state 20")
            else:
                print("localizing..")
                hover_goal = drone_pose_map
                state = 15
        
        if state == 15: # wait for spin or localize
            drone_mode = 1 # velocity control
            publish_vel_cmd(hover_goal, 0, 0, 50)

            if is_localized:
                state = 20
                drone_mode = 0
                current_waypoint = drone_pose_map
                current_waypoint_index = 0
                print("localized, go to state 20")

        if state == 20: # get next goal for exploration
            rospy.wait_for_service('explorer_request_goal')
            explorer_srv = rospy.ServiceProxy('explorer_request_goal', ExploreReqGoal)
            result = explorer_srv()
            goal = result.next_goal
            print(goal)

            state = 30
            print("set new goal for exploration, go to state 30")

        if state == 30: # plan the path
            # print("planning path")
            drone_pose = rospy.wait_for_message('/cf1/pose', PoseStamped)
            # print("drone_pose odom: ", drone_pose)
            # transform cf pose to map frame, return PoseStamped msg
            start = trans2Map(drone_pose)

            # print("start: ", start)
            # print("goal: ", goal)

            # call planning service for list of pose stamped msg between start and goal
            rospy.wait_for_service('drone_path')
            planning_srv = rospy.ServiceProxy('drone_path', DronePath)
            path = planning_srv(start, goal)
            path = path.path

            if len(path.poses) == 0:
                # no path found, go back to explore
                # current_waypoint_index = -1
                state = 20
                print("no path found, go to state 20")
            else:
                path_pub.publish(path)
                current_waypoint_index = 0
                state = 40
                print("path found, go to state 40")

        if state == 40: # move
            if not is_localized:
                state = 10
                print("not loc in state 40, go to state 10")


            current_waypoint = path.poses[current_waypoint_index]

            publish_pos_cmd(current_waypoint)
            path_pub.publish(path)

            # print("drone pose: ", drone_pose_map)
            # print("current waypoint: ", current_waypoint)

            # check if waypoint reached
            distance_to_goal, yaw_dist = check_distance_to_goal(drone_pose_map, current_waypoint)
            print("distance, yaw_dist: ", distance_to_goal, yaw_dist)

            if distance_to_goal < 0.15 and yaw_dist < 20:
                # move to next setpont or start new exploration goal
                if current_waypoint_index < len(path.poses) - 1:
                    current_waypoint_index += 1 
                else:
                    state = 50
                    # explore point
                    rospy.wait_for_service('explorer_explore_point')
                    explorer_srv = rospy.ServiceProxy('explorer_explore_point', ExplorePoint)
                    explorer_srv(current_waypoint)
                    print("reached final waypoint, go to state 50")

        if state == 50: # spin at final waypoint
            _, _, last_yaw = euler_from_quaternion((drone_pose_map.pose.orientation.x, 
                                                            drone_pose_map.pose.orientation.y, 
                                                            drone_pose_map.pose.orientation.z, 
                                                            drone_pose_map.pose.orientation.w))
            total_angle = 0
            hover_goal = drone_pose_map
            state = 55
            print("spinning at final waypoint..")
        
        if state == 55:
            drone_mode = 1 # velocity control
            publish_vel_cmd(hover_goal, 0, 0, 50)
            _, _, current_yaw = euler_from_quaternion((drone_pose_map.pose.orientation.x, 
                                                            drone_pose_map.pose.orientation.y, 
                                                            drone_pose_map.pose.orientation.z, 
                                                            drone_pose_map.pose.orientation.w))
            delta = math.atan2(math.sin(current_yaw-last_yaw), math.cos(current_yaw-last_yaw))
            delta = math.degrees(delta)
            # print("last yaw, current yaw, delta: ", math.degrees(last_yaw), math.degrees(current_yaw), delta)
            total_angle += delta
            # print("total_angle: ", total_angle)
            last_yaw = current_yaw

            if abs(total_angle) >= 360:
                print("spinning at waypoint done, go to state 20")
                drone_mode = 0
                current_waypoint = drone_pose_map
                current_waypoint_index = 0
                state = 20

        if state == 100: # intruder found
            print("=================================")
            print("======FOUND THE INTRUDER!========")
            print("=================================")
            done = True
            state = 1000 # do nothing but hover at last position
        


        rate.sleep()

if __name__ == "__main__":
    main()