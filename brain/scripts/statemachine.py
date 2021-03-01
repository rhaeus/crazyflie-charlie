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

set_points = None
set_point_index = 0
set_point_count = 0

def move_callback(msg):
    global set_points
    global set_point_count
    global set_point_index

    

    # cf1/pose is in odom frame
    # set point goal is in  map frame
    # transform pose to map frame in order to compare poses
    # msg.header.stamp = rospy.Time.now()
    if not tf_buf.can_transform('map', msg.header.frame_id, msg.header.stamp):
    # if not tf_buf.can_transform('map', msg.header.frame_id, rospy.Time(0)):
        # rospy.logwarn_throttle(5.0, 'No transform from %s to map' % msg.header.frame_id)
        rospy.logwarn('No transform from %s to map' % msg.header.frame_id)
        return

    pose_map = tf_buf.transform(msg, 'map')

    distance_to_goal = np.sqrt(math.pow(pose_map.pose.position.x - set_points[set_point_index].pose.position.x,2) 
    + math.pow(pose_map.pose.position.y - set_points[set_point_index].pose.position.y, 2) 
    + math.pow(pose_map.pose.position.z - set_points[set_point_index].pose.position.z, 2))

    msg_roll, msg_pitch, msg_yaw = euler_from_quaternion((pose_map.pose.orientation.x,
                                              pose_map.pose.orientation.y,
                                              pose_map.pose.orientation.z,
                                              pose_map.pose.orientation.w))

    goal_roll, goal_pitch, goal_yaw = euler_from_quaternion((set_points[set_point_index].pose.orientation.x,
                                              set_points[set_point_index].pose.orientation.y,
                                              set_points[set_point_index].pose.orientation.z,
                                              set_points[set_point_index].pose.orientation.w))

    roll_dist = abs(math.degrees(msg_roll) - math.degrees(goal_roll))
    pitch_dist = abs(math.degrees(msg_pitch) - math.degrees(goal_pitch))
    yaw_dist = abs(math.degrees(msg_yaw) - math.degrees(goal_yaw))



    if distance_to_goal < 0.1 and yaw_dist < 10:
        # move to next setpoint or stay at last point
        if set_point_index < set_point_count - 1:
            set_point_index = (set_point_index + 1) 
        # set_point_index = set_point_index % set_point_count

    publish_cmd(set_points[set_point_index])

def publish_cmd(goal):
    # Need to tell TF that the goal was just generated
    goal.header.stamp = rospy.Time.now()

    # goal is in map frame
    # drone expects goal pose in odom frame
    if not tf_buf.can_transform('cf1/odom', goal.header.frame_id, goal.header.stamp):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
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
    

# def pose_stamped_from_marker(m):
#     p = PoseStamped()
#     p.header.seq = str(m['id'])
#     p.header.stamp = rospy.Time(0)
#     p.header.frame_id = 'map'

#     p.pose.position = Vector3(*m['pose']['position'])
#     roll, pitch, yaw = m['pose']['orientation']
#     (p.pose.orientation.x,
#      p.pose.orientation.y,
#      p.pose.orientation.z,
#      p.pose.orientation.w) = quaternion_from_euler(math.radians(roll),
#                                                      math.radians(pitch),
#                                                      math.radians(yaw))
#     return p

# def transform_from_marker(m):
#     t = TransformStamped()
#     t.header.frame_id = 'map'
#     t.child_frame_id = 'setpoint' + str(m['id'])
#     t.transform.translation = Vector3(*m['pose']['position'])
#     roll, pitch, yaw = m['pose']['orientation']
#     (t.transform.rotation.x,
#      t.transform.rotation.y,
#      t.transform.rotation.z,
#      t.transform.rotation.w) = quaternion_from_euler(math.radians(roll),
#                                                      math.radians(pitch),
#                                                      math.radians(yaw))
#     return t

global is_localized
is_localized = False

def flagcallback(msg):
    global is_localized
    is_localized = msg.data

rospy.init_node('move_drone')

pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
# sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, move_callback)

sub_flag = rospy.Subscriber("is_localized", Bool, flagcallback)




tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def trans2Map(msg):
    # marker pose is in frame camera_link
    if not tf_buf.can_transform('map', msg.header.frame_id, msg.header.stamp, rospy.Duration(1)):
        rospy.logwarn('No transform from %s to map', msg.header.frame_id)
        return
    # transform = tf_buf.lookup_transform('map',msg.header.frame_id,msg.header.stamp,rospy.Duration(1))
    # mapPose = tf2_geometry_msgs.do_transform_pose(msg,transform)
    return tf_buf.transform(msg, 'map')


def main(argv=sys.argv):
    state = 0

    while not rospy.is_shutdown():
        
        if state == 0: # localize
            print("wait for localize")
            global is_localized
            if is_localized:
                print("localized")
                state = 1

        elif state == 1: # plan path
            print("getting path")
            drone_pose = rospy.wait_for_message('/cf1/pose', PoseStamped)
            # transform cf pose to map frame, return PoseStamped msg
            # start = trans2Map(drone_pose)
            start = PoseStamped()
            start.header.frame_id = 'map'
            start.pose.position.x = 0
            start.pose.position.y = 0
            start.pose.position.z = 0
            (start.pose.orientation.x,
            start.pose.orientation.y,
            start.pose.orientation.z,
            start.pose.orientation.w) = quaternion_from_euler(0,0,0)

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = 1.5
            goal.pose.position.y = 0.5
            goal.pose.position.z = 0
            (goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w) = quaternion_from_euler(0,0,0)

            print("start: ", start)
            print("goal: ", goal)

            # call planning service for list of pose stamped msg between start and goal
            rospy.wait_for_service('drone_path')
            planning_srv = rospy.ServiceProxy('drone_path', DronePath)
            path = planning_srv(start, goal)
            set_points = path
            set_point_count = len(set_points)

            state = 3

        elif state == 3: # get ready to move to points
            print("get ready to move to points")
            sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, move_callback)
            state = 4

        elif state == 4: # it should move
            state = 4 

    
    # is_localized = False
    # # wait for localize
    # while not is_localized:
    #     print("not loc")
    #     rospy.spin()

    
    # print("after loc")


    # # get path 
    # #transform cf pose to map frame, return PoseStamped msg
    # start = trans2Map(drone_pose)
    # # get goal pose in map frame, return PoseStamed msg
    # goal = PoseStamped()
    # goal.header.frame_id = 'map'
    # goal.pose.position.x = 1.5
    # goal.pose.position.y = 0.5
    # goal.pose.position.z = 0
    # (goal.pose.orientation.x,
    # goal.pose.orientation.y,
    # goal.pose.orientation.z,
    # goal.pose.orientation.w) =tf.transformations.quaternion_from_euler(0,0,0)
    # # call planning service for list of pose stamped msg between start and goal
    # path = planning(start, goal)
   

    # # create a pose stamped for each setpoint and store in list   
    # global set_points
    # set_points = [pose_stamped_from_marker(m) for m in points['markers']]

    # # Create a transform for each setpoint and publish it as tf
    # transforms = [transform_from_marker(m) for m in points['markers']]
    # broadcaster = tf2_ros.StaticTransformBroadcaster()
    # broadcaster.sendTransform(transforms)

    # global set_point_count
    # set_point_count = len(set_points)

    rospy.spin()

# def main(argv=sys.argv):
#     # Let ROS filter through the arguments
#     args = rospy.myargv(argv=argv)

#     # Load setpoints from json file that is given as argument
#     with open(args[1], 'rb') as f:
#         points = json.load(f)

#     # create a pose stamped for each setpoint and store in list   
#     global set_points
#     set_points = [pose_stamped_from_marker(m) for m in points['markers']]

#     # Create a transform for each setpoint and publish it as tf
#     transforms = [transform_from_marker(m) for m in points['markers']]
#     broadcaster = tf2_ros.StaticTransformBroadcaster()
#     broadcaster.sendTransform(transforms)

#     global set_point_count
#     set_point_count = len(set_points)

#     rospy.spin()

if __name__ == "__main__":
    main()