#!/usr/bin/env python

import json
import sys
import math

import rospy
import numpy as np

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Vector3, PoseArray, Pose, TransformStamped
from crazyflie_driver.msg import Position

import tf2_ros
import tf2_geometry_msgs

set_points = None
set_point_index = 0
set_point_count = 0

def pose_callback(msg):
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
        set_point_index = (set_point_index + 1) 
        set_point_index = set_point_index % set_point_count

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

def publish_setpoint_poses():
    posearray = PoseArray()
    posearray.header.stamp = rospy.Time.now()
    posearray.header.frame_id = 'map'
    

    global set_points
    for point in set_points:
        posearray.poses.append(point.pose)

    pub_setpoints.publish(posearray)
    

def pose_stamped_from_marker(m):
    p = PoseStamped()
    p.header.seq = str(m['id'])
    p.header.stamp = rospy.Time(0)
    p.header.frame_id = 'map'

    p.pose.position = Vector3(*m['pose']['position'])
    roll, pitch, yaw = m['pose']['orientation']
    (p.pose.orientation.x,
     p.pose.orientation.y,
     p.pose.orientation.z,
     p.pose.orientation.w) = quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))
    return p

def transform_from_marker(m):
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'setpoint' + str(m['id'])
    t.transform.translation = Vector3(*m['pose']['position'])
    roll, pitch, yaw = m['pose']['orientation']
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))
    return t

rospy.init_node('move_drone')

pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
# pub_setpoints  = rospy.Publisher('/setpoint_poses', PoseArray, queue_size=2)
sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)


tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)




def main(argv=sys.argv):
    # Let ROS filter through the arguments
    args = rospy.myargv(argv=argv)

    # Load world JSON
    with open(args[1], 'rb') as f:
        points = json.load(f)

    # Create a transform for each marker
    global set_points
    set_points = [pose_stamped_from_marker(m) for m in points['markers']]

    transforms = [transform_from_marker(m) for m in points['markers']]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)

    global set_point_count
    set_point_count = len(set_points)

    # publish_setpoint_poses()


    # rospy.loginfo(set_points)
    rospy.spin()

if __name__ == "__main__":
    main()