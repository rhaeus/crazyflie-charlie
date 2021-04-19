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

class Brain:
    def __init__(self):
        self.is_localized = False
        self.drone_pose_map = PoseStamped()
        self.intruder_found = False

        self.current_waypoint = PoseStamped()
        self.current_waypoint_index = -1
        self.drone_mode = 0 # drone control mode, 0=position, 1=velocity

        self.tf_buf   = tf2_ros.Buffer()
        self.tf_lstn  = tf2_ros.TransformListener(self.tf_buf)

        self.explorer_request_goal = rospy.ServiceProxy('explorer_request_goal', ExploreReqGoal)
        rospy.wait_for_service('explorer_request_goal')
        self.planning_srv = rospy.ServiceProxy('drone_path', DronePath)
        rospy.wait_for_service('drone_path')
        self.explorer_explore_point = rospy.ServiceProxy('explorer_explore_point', ExplorePoint)
        rospy.wait_for_service('explorer_explore_point')

        self.sub_flag = rospy.Subscriber("is_localized", Bool, self.localize_flag_callback)
        self.sub_pose = rospy.Subscriber("/cf1/pose", PoseStamped, self.pose_callback)
        self.sub_intruder = rospy.Subscriber("/cf1/intruder_detection_result", Bool, self.intruder_callback)

        self.pub_pos_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
        self.pub_vel_cmd  = rospy.Publisher('/cf1/cmd_hover', Hover, queue_size=2)
        self.pub_path = rospy.Publisher('/cf1/path_pub', Path, queue_size=10)
        self.pub_safe_zone = rospy.Publisher('/cf1/safe_zone', Bool, queue_size=10)

        rospy.sleep(3) #give ROS some time to setup stuff


    def localize_flag_callback(self, msg):
        self.is_localized = msg.data

    def trans2Map(self, msg):
        # marker pose is in frame camera_link
        if not self.tf_buf.can_transform('map', msg.header.frame_id, msg.header.stamp, rospy.Duration(1)):
            rospy.logwarn('[brain][trans2Map] No transform from %s to map', msg.header.frame_id)
            return
        return self.tf_buf.transform(msg, 'map')

    def publish_pos_cmd(self, goal):
        # Need to tell TF that the goal was just generated
        goal.header.stamp = rospy.Time(0)
        # goal.header.stamp = rospy.Time.now()

        # goal is in map frame
        # drone expects goal pose in odom frame
        if not self.tf_buf.can_transform('cf1/odom', goal.header.frame_id, goal.header.stamp, rospy.Duration(1)):
            rospy.logwarn('[brain][publish_pos_cmd] No transform from %s to cf1/odom' % goal.header.frame_id)
            return

        goal_odom = self.tf_buf.transform(goal, 'cf1/odom')

        cmd = Position()

        cmd.header.stamp = goal_odom.header.stamp
        cmd.header.frame_id = goal_odom.header.frame_id

        cmd.x = goal_odom.pose.position.x
        cmd.y = goal_odom.pose.position.y
        cmd.z = goal_odom.pose.position.z

        _, _, yaw = euler_from_quaternion((goal_odom.pose.orientation.x,
                                                goal_odom.pose.orientation.y,
                                                goal_odom.pose.orientation.z,
                                                goal_odom.pose.orientation.w))

        cmd.yaw = math.degrees(yaw)

        self.pub_pos_cmd.publish(cmd)

    def publish_vel_cmd(self, goal, vx, vy, vyaw):
        # Need to tell TF that the goal was just generated
        goal.header.stamp = rospy.Time(0)
        # goal.header.stamp = rospy.Time.now()

        # goal is in map frame
        # drone expects goal pose in odom frame
        if not self.tf_buf.can_transform('cf1/odom', goal.header.frame_id, goal.header.stamp, rospy.Duration(1)):
            rospy.logwarn('[brain][publish_pos_cmd] No transform from %s to cf1/odom' % goal.header.frame_id)
            return

        goal_odom = self.tf_buf.transform(goal, 'cf1/odom')

        cmd = Hover()
        cmd.header.stamp = goal_odom.header.stamp
        cmd.header.frame_id = goal_odom.header.frame_id
        cmd.vx = vx
        cmd.vy = vy
        cmd.yawrate = vyaw
        # print("vx, vy, vyaw:", vx, vy, vyaw)
        cmd.zDistance = goal_odom.pose.position.z

        self.pub_vel_cmd.publish(cmd)

    def check_distance_to_goal(self, pose_map, goal_map):

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

    def pose_callback(self, msg):
        self.drone_pose_map = self.trans2Map(msg)

        # keep drone flying even if statemachine waits for path planning
        # this callback is called anyways
        if self.drone_mode == 0 and self.current_waypoint_index != -1:
            self.publish_pos_cmd(self.current_waypoint)

    def intruder_callback(self, msg):
        self.intruder_found = msg.data

    def run_statemachine(self):
        rate = rospy.Rate(10)

        state = 0
        path = []
        goal = PoseStamped()
        self.is_localized = False
        done = False
        # safe_zone = True # start position is safe zone becase we start where we can see a marker
        # self.pub_safe_zone.publish(safe_zone)

        print("start statemachine")


        while not rospy.is_shutdown():

            # publish position command if valid to prevent drone from landing
            if self.drone_mode == 0 and self.current_waypoint_index != -1:
                self.publish_pos_cmd(self.current_waypoint)

            # # check if intruder is found
            if not done and self.intruder_found: 
                self.drone_mode = 0
                self.current_waypoint_index = 0
                self.current_waypoint = self.drone_pose_map
                state = 100

            if state == 0: # initial localize
                if self.is_localized:
                    state = 10
                    print("localized, go to state 10")
                else:
                    print("localizing..")
                    # hover_goal = self.drone_pose_map
                    # self.current_waypoint = self.drone_pose_map
                    # self.current_waypoint_index = 0
                    safe_zone = True # start position is safe zone becase we start where we can see a marker
                    self.pub_safe_zone.publish(safe_zone)
                    state = 5
            
            if state == 5: # wait for spin or localize
                # self.drone_mode = 1 # velocity control
                # self.publish_vel_cmd(hover_goal, 0, 0, 50)
                # self.publish_pos_cmd(self.current_waypoint)

                if self.is_localized:
                    state = 10
                    # self.drone_mode = 0
                    # self.current_waypoint = self.drone_pose_map
                    # self.current_waypoint_index = 0
                    safe_zone = False
                    self.pub_safe_zone.publish(False)
                    print("localized, go to state 10")

            if state == 10: # startup
                # liftoff
                drone_pose = rospy.wait_for_message('/cf1/pose', PoseStamped)
                start = self.trans2Map(drone_pose)
                if start is not None:
                    start.pose.position.z = 0.4
                    self.current_waypoint_index = 0
                    self.current_waypoint = start
                    self.publish_pos_cmd(self.current_waypoint)
                    state = 15
                    print("liftoff")

            if state == 15: #wait for liftoff
                self.publish_pos_cmd(self.current_waypoint)
                if abs(self.drone_pose_map.pose.position.z - 0.4) < 0.05:
                    state =  20
                    print("startup done, go to state 20")

            if state == 20: # get next goal for exploration
                result = self.explorer_request_goal()
                goal = result.next_goal
                safe_zone = result.is_safe_zone.data
                print(goal)

                state = 30
                print("set new goal for exploration, go to state 30")

            if state == 30: # plan the path
                # print("planning path")
                drone_pose = rospy.wait_for_message('/cf1/pose', PoseStamped)
                # print("drone_pose odom: ", drone_pose)
                # transform cf pose to map frame, return PoseStamped msg
                start = self.trans2Map(drone_pose)

                # print("start: ", start)
                # print("goal: ", goal)

                # call planning service for list of pose stamped msg between start and goal
                path = self.planning_srv(start, goal)
                path = path.path

                if len(path.poses) == 0:
                    # no path found, go back to explore
                    # current_waypoint_index = -1
                    state = 20
                    print("no path found, go to state 20")
                else:
                    self.pub_path.publish(path)
                    self.current_waypoint_index = 0
                    state = 40
                    print("path found, go to state 40")

            if state == 40: # move
                # if not self.is_localized:
                #     state = 10
                #     print("not loc in state 40, go to state 10")


                self.current_waypoint = path.poses[self.current_waypoint_index]

                self.publish_pos_cmd(self.current_waypoint)
                self.pub_path.publish(path)

                # print("drone pose: ", drone_pose_map)
                # print("current waypoint: ", current_waypoint)

                # check if waypoint reached
                distance_to_goal, yaw_dist = self.check_distance_to_goal(self.drone_pose_map, self.current_waypoint)
                # print("distance, yaw_dist: ", distance_to_goal, yaw_dist)

                if distance_to_goal < 0.15 and yaw_dist < 20:
                    # move to next setpont or start new exploration goal
                    if self.current_waypoint_index < len(path.poses) - 1:
                        self.current_waypoint_index += 1 
                    else:
                        # explore point
                        self.explorer_explore_point(self.current_waypoint)
                        if safe_zone:
                            print("reached final waypoint in safe zone, go to state 50")
                            state = 50
                        else:
                            print("reached final waypoint, go to state 60")
                            state = 60

            if state == 50: # wait in safe spot
                # loop runs with 10hz = 10times/s -> one run takes 1/10s
                # we want to wait 3s in safe zone
                # therefore safe_count must be 30
                # safe_count = 100000000000000
                self.pub_safe_zone.publish(True)
                state = 55
                print("waiting in safe zone...")
            
            if state == 55:
                if self.is_localized:
                    self.pub_safe_zone.publish(False)
                    state = 60
                    print("waiting done, go to state 60")

                # safe_count -= 1
                # if safe_count <= 0:
                #     self.pub_safe_zone.publish(False)
                #     state = 60
                #     print("waiting done, go to state 20")

            if state == 60: # spin at final waypoint
                _, _, last_yaw = euler_from_quaternion((self.drone_pose_map.pose.orientation.x, 
                                                                self.drone_pose_map.pose.orientation.y, 
                                                                self.drone_pose_map.pose.orientation.z, 
                                                                self.drone_pose_map.pose.orientation.w))
                total_angle = 0
                self.current_waypoint = self.drone_pose_map
                self.current_waypoint_index = 0

                self.drone_mode = 0 # position control

                # self.publish_pos_cmd(self.current_waypoint)
                state = 65
                print("spinning at final waypoint..")
            
            if state == 65:
                self.publish_pos_cmd(self.current_waypoint)

                distance_to_goal, yaw_dist = self.check_distance_to_goal(self.drone_pose_map, self.current_waypoint)
                # print("distance, yaw_dist: ", distance_to_goal, yaw_dist)

                if distance_to_goal < 0.15 and yaw_dist < 5:
                    _, _, current_yaw = euler_from_quaternion((self.drone_pose_map.pose.orientation.x, 
                                                                self.drone_pose_map.pose.orientation.y, 
                                                                self.drone_pose_map.pose.orientation.z, 
                                                                self.drone_pose_map.pose.orientation.w))

                    delta = math.atan2(math.sin(current_yaw-last_yaw), math.cos(current_yaw-last_yaw))
                    delta = math.degrees(delta)
                    # print("last yaw, current yaw, delta: ", math.degrees(last_yaw), math.degrees(current_yaw), delta)
                    total_angle += delta
                    # print("total_angle: ", total_angle)
                    last_yaw = current_yaw

                    if abs(total_angle) >= 360:
                        print("total angle: ", total_angle)
                        print("spinning at waypoint done, go to state 70")
                        self.current_waypoint = self.drone_pose_map
                        self.current_waypoint_index = 0
                        state = 70
                    else:
                        angle_to_go = 360 - total_angle
                        if angle_to_go > 20:
                            angle_to_go = 20
                        new_yaw = current_yaw + math.radians(angle_to_go)
                        if new_yaw > math.pi:
                            new_yaw -= math.pi * 2.0
                        if new_yaw < -math.pi:
                            new_yaw += math.pi * 2.0

                        (self.current_waypoint.pose.orientation.x, 
                        self.current_waypoint.pose.orientation.y,
                        self.current_waypoint.pose.orientation.z,
                        self.current_waypoint.pose.orientation.w) = quaternion_from_euler(0, 0, new_yaw)


            if state == 70: # wait in safe spot
                # loop runs with 10hz = 10times/s -> one run takes 1/10s
                # we want to wait 3s in safe zone
                # therefore safe_count must be 30
                # safe_count = 100000000000000
                self.pub_safe_zone.publish(True)
                state = 75
                print("waiting in safe zone...")
            
            if state == 75:
                if self.is_localized:
                    self.pub_safe_zone.publish(False)
                    state = 20
                    print("waiting done, go to state 20")

                # safe_count -= 1
                # if safe_count <= 0:
                #     self.pub_safe_zone.publish(False)
                #     state = 60
                #     print("waiting done, go to state 20")

            if state == 100: # intruder found
                print("=================================")
                print("======FOUND THE INTRUDER!========")
                print("=================================")
                done = True
                state = 1000 # do nothing but hover at last position
            


            rate.sleep()

def main():
    rospy.init_node('brain')
    brain = Brain()
    brain.run_statemachine()

if __name__ == "__main__":
    main()