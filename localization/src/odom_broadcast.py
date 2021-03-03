#!/usr/bin/env python

import sys 
import json 
import numpy as np
import tf2_geometry_msgs

import math
import rospy
import tf2_ros
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, PoseStamped, Transform
from std_msgs.msg import Bool
import tf.transformations 
"""
def marker_callback(msg):
    for marker in msg.markers:
        #rospy.loginfo(marker)
        broadcast_odom(marker)
        trans_to_map(marker)

def poseToMatrix(tran, quat):
    T = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    T[0,3]= tran.x
    T[1,3]= tran.y
    T[2,3]= tran.z
    return T 

def matrixToPose(T):
    tran = tf.transformations.translation_from_matrix(T)
    quat = tf.transformations.quaternion_from_matrix(T)
    return tran, quat 

def mMapMatrix(m):
    markers = world['markers']
    for marker in markers:
        if marker['id'] == m.id:
            trans = tf.transformations.translation_matrix(marker['pose']['position'])
            rot = marker['pose']['orientation']
            T = tf.transformations.euler_matrix(rot[0], rot[1], rot[2])
            T[0,3] = trans[0,3]
            T[1,3] = trans[1,3]
            T[2,3] = trans[2,3]
            continue
    
    return T 



def broadcast_odom(m):
    # cam to landmark
    c_T_l = poseToMatrix(m.pose.pose.position, m.pose.pose.orientation)

    #baselink to cam
    t = tf_buf.lookup_transform('cf1/base_link', 'cf1/camera_link', m.header.stamp)
    b_T_c  = poseToMatrix(t.transform.translation, t.transform.rotation)
    
    b_T_l = np.matmul(b_T_c, c_T_l)

    l_T_b = tf.transformations.inverse_matrix(b_T_l)

    #map to landmark
    m_T_l = mMapMatrix(m)

    m_T_b = np.matmul(m_T_l, l_T_b)

    # baselink to odom
    if not tf_buf.can_transform('cf1/base_link', 'cf1/odom', m.header.stamp, timeout = rospy.Duration(0.1)):
        return
    t = tf_buf.lookup_transform('cf1/base_link', 'cf1/odom', m.header.stamp, timeout=rospy.Duration(0.1))
    
    b_T_o = poseToMatrix(t.transform.translation, t.transform.rotation)

    #map to odom    
    m_T_o = np.matmul(m_T_b, b_T_o)
    #extract translation and rotation 
    trans, quat = matrixToPose(m_T_o)

    # set up transform message
    t = TransformStamped()
    t.header.stamp = m.header.stamp
    t.header.frame_id = 'map'
    t.child_frame_id = 'cf1/odom'
    t.transform.translation.x = trans[0]
    t.transform.translation.y = trans[1]
    t.transform.translation.z = 0

    norm = np.sqrt(quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2)

    angles = tf.transformations.euler_from_quaternion([quat[0],quat[1],quat[2],quat[3]])

    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) =tf.transformations.quaternion_from_euler(0,0,angles[2])


    broadcaster.sendTransform(t)

    if tf_buf.can_transform('cf1/odom','map',m.header.stamp, timeout=rospy.Duration(0.1)):
        msg = Bool()
        msg.data = True
        pub.publish(msg)
    else:
        msg = Bool()
        msg.data = False
        pub.publish(msg)
    



def trans_to_map(m):
    aruco_marker = PoseStamped()
    aruco_marker.header = m.header
    aruco_marker.pose = m.pose.pose 

    if not tf_buf.can_transform('map', aruco_marker.header.frame_id, aruco_marker.header.stamp, timeout = rospy.Duration(0.1)):
        return 

    aruco_marker = tf_buf.transform(aruco_marker, 'map')

    trans = TransformStamped()
    trans.header.stamp = m.header.stamp 
    trans.header.frame_id = 'map'
    trans.child_frame_id = 'aruco/detected' + str(m.id)
    trans.transform = Transform(translation=aruco_marker.pose.position, rotation=aruco_marker.pose.orientation)

    broadcaster.sendTransform(trans)

def main(argv=sys.argv): 
    rospy.init_node('marker_detection')
    global tf_buf, broadcaster, world, pub

    path = str(argv[1])
    with open(path, 'rb') as f:
        world = json.load(f)  

    pub = rospy.Publisher('is_localized', Bool, queue_size=10)
    sub_marker = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)

    tf_buf = tf2_ros.Buffer() 
    tf_lstn = tf2_ros.TransformListener(tf_buf)
    broadcaster = tf2_ros.TransformBroadcaster()
    rospy.spin()

if __name__ == "__main__":
    main()
"""


marker = None 

def marker_callback(msg):
    global marker
    for marker in msg.markers:
        broadcast_odom(marker)
        #trans_to_map(marker)


def read_static_marker(m):
    markers = world['markers']
    for marker in markers:
        if marker['id'] == m.id:
            trans = marker['pose']['position']
            rot = marker['pose']['orientation']

    return trans, rot


def trans_to_map(m):
    aruco_marker = PoseStamped()
    aruco_marker.header = m.header
    aruco_marker.pose = m.pose.pose 

    if not tf_buf.can_transform('map', aruco_marker.header.frame_id, aruco_marker.header.stamp, timeout = rospy.Duration(0.1)):
        return 

    aruco_marker = tf_buf.transform(aruco_marker, 'map')

    trans = TransformStamped()
    trans.header.stamp = m.header.stamp 
    trans.header.frame_id = 'map'
    trans.child_frame_id = 'aruco/detected' + str(m.id)
    trans.transform = Transform(translation=aruco_marker.pose.position, rotation=aruco_marker.pose.orientation)

    broadcaster.sendTransform(trans)

def broadcast_odom(m):
    marker_pose = PoseStamped()
    marker_pose.header = m.header
    marker_pose.pose = m.pose.pose 

    if not tf_buf.can_transform('cf1/odom', 'cf1/camera_link', m.header.stamp, timeout=rospy.Duration(0.1)):
        return 

    camera_in_odom = tf_buf.lookup_transform('cf1/odom', 'cf1/camera_link', m.header.stamp, timeout=rospy.Duration(0.1))
    marker_in_odom = tf2_geometry_msgs.do_transform_pose(marker_pose, camera_in_odom)     
    
    odom_translation_marker = marker_in_odom.pose.position
    odom_rotation_marker = marker_in_odom.pose.orientation
    odom_rotation_marker_list = [odom_rotation_marker.x, odom_rotation_marker.y, odom_rotation_marker.z, odom_rotation_marker.w]
    odom_rotation_marker_list_inv = tf.transformations.quaternion_inverse(odom_rotation_marker_list)

    map_translation_marker, map_rotation_marker = read_static_marker(m)   

    map_rotation_marker = tf.transformations.quaternion_from_euler(map_rotation_marker[0] * math.pi/180, map_rotation_marker[1] * math.pi/180, map_rotation_marker[2] * math.pi/180)
    map_rotation_odom = tf.transformations.quaternion_multiply(map_rotation_marker, odom_rotation_marker_list_inv)
    
    euler_angles = tf.transformations.euler_from_quaternion(map_rotation_odom)
    
    trans = TransformStamped()
    trans.header.stamp = m.header.stamp
    trans.header.frame_id = 'map'
    trans.child_frame_id = 'cf1/odom'
    trans.transform.translation.x = map_translation_marker[0] - odom_translation_marker.x
    trans.transform.translation.y = map_translation_marker[1] - odom_translation_marker.y
    trans.transform.translation.z = 0
    (trans.transform.rotation.x,
     trans.transform.rotation.y,
     trans.transform.rotation.z,
     trans.transform.rotation.w) =tf.transformations.quaternion_from_euler(0,0,euler_angles[2])
    
    broadcaster.sendTransform(trans)


rospy.init_node('marker_detection')
sub_marker = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)

tf_buf = tf2_ros.Buffer() 
tf_lstn = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()

def main(argv=sys.argv):
    rospy.init_node('marker_detection')
    global world

    path = str(argv[1])
    with open(path, 'rb') as f:
        world = json.load(f)  

    #pub = rospy.Publisher('is_localized', Bool, queue_size=10)
    #sub_marker = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if marker:
            broadcast_odom(marker)

        rate.sleep()
    #rospy.spin()


if __name__ == "__main__":
    main()