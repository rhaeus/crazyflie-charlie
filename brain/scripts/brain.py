#!/usr/bin/env python

import json
import sys
import math

import rospy
import numpy as np

from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Vector3, PoseArray, Pose, TransformStamped
from crazyflie_driver.msg import Position

import tf2_ros
import tf2_geometry_msgs

from cf_msgs.srv import DronePath
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

def publish_cmd(goal):
    # Need to tell TF that the goal was just generated
    goal.header.stamp = rospy.Time.now()

    # goal is in map frame
    # drone expects goal pose in odom frame
    if not tf_buf.can_transform('cf1/odom', goal.header.frame_id, goal.header.stamp, rospy.Duration(1)):
        rospy.logwarn('[publish_cmd] No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    goal_odom = tf_buf.transform(goal, 'cf1/odom')

    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = goal_odom.header.frame_id

    cmd.x = goal_odom.pose.position.x
    cmd.y = goal_odom.pose.position.y
    cmd.z = goal_odom.pose.position.z

    roll, pitch, yaw = euler_from_quaternion((goal_odom.pose.orientation.x,
                                              goal_odom.pose.orientation.y,
                                              goal_odom.pose.orientation.z,
                                              goal_odom.pose.orientation.w))

    cmd.yaw = math.degrees(yaw)

    pub_cmd.publish(cmd)


def check_distance_to_goal(pose_map, goal_map):

    distance_to_goal = np.sqrt(math.pow(pose_map.pose.position.x - goal_map.pose.position.x,2) 
    + math.pow(pose_map.pose.position.y - goal_map.pose.position.y, 2) 
    + math.pow(pose_map.pose.position.z - goal_map.pose.position.z, 2))

    msg_roll, msg_pitch, msg_yaw = euler_from_quaternion((pose_map.pose.orientation.x,
                                              pose_map.pose.orientation.y,
                                              pose_map.pose.orientation.z,
                                              pose_map.pose.orientation.w))

    goal_roll, goal_pitch, goal_yaw = euler_from_quaternion((goal_map.pose.orientation.x,
                                              goal_map.pose.orientation.y,
                                              goal_map.pose.orientation.z,
                                              goal_map.pose.orientation.w))

    roll_dist = abs(math.degrees(msg_roll) - math.degrees(goal_roll))
    pitch_dist = abs(math.degrees(msg_pitch) - math.degrees(goal_pitch))
    yaw_dist = abs(math.degrees(msg_yaw) - math.degrees(goal_yaw))

    return distance_to_goal, roll_dist, pitch_dist, yaw_dist

def pose_callback(msg):
    global drone_pose_map
    drone_pose_map = trans2Map(msg)


rospy.init_node('brain')

global is_localized
is_localized = False

global drone_pose_map

sub_flag = rospy.Subscriber("is_localized", Bool, localize_flag_callback)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
sub_pose = rospy.Subscriber("/cf1/pose", PoseStamped, pose_callback)
path_pub = rospy.Publisher('/path_pub', Path, queue_size=10)

tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def main(argv=sys.argv):
    rate = rospy.Rate(10)

    state = 0
    path = []
    current_waypoint_index = 0
    goal = PoseStamped()
    global is_localized
    is_localized = True

    while not rospy.is_shutdown():
        
        if state == 0: # startup
            # liftoff
            state = 10
            print("startup done, go to state 10")
        
        if state == 10: # localize
            # spin if not localized
            if is_localized:
                state = 20
                print("localized, go to state 20")

        if state == 20: # get next goal for exploration
            goal.header.frame_id = 'map'
            goal.pose.position.x = 8
            goal.pose.position.y = 0
            goal.pose.position.z = 0
            (goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w) = quaternion_from_euler(0,0,0)

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
            current_waypoint_index = 0

            if len(path.poses) == 0:
                # no path found, go back to explore
                state = 20
                print("no path found, go to state 20")
            else:
                path_pub.publish(path)
                state = 40
                print("path found, go to state 40")

        if state == 40: # move
            if not is_localized:
                state = 10
                print("not loc in state 40, go to state 10")


            current_waypoint = path.poses[current_waypoint_index]

            publish_cmd(current_waypoint)
            path_pub.publish(path)

            # print("drone pose: ", drone_pose_map)
            # print("current waypoint: ", current_waypoint)

            # check if waypoint reached
            distance_to_goal, roll_dist, pitch_dist, yaw_dist = check_distance_to_goal(drone_pose_map, current_waypoint)

            if distance_to_goal < 0.15 and yaw_dist < 10:
                # move to next setpont or start new exploration goal
                if current_waypoint_index < len(path.poses) - 1:
                    current_waypoint_index += 1 
                else:
                    state = 20
                    print("reached final waypoint, go to state 20")


        rate.sleep()

if __name__ == "__main__":
    main()