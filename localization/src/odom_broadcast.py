#!/usr/bin/env python

import sys 
import json 
sys.path.append('/home/karl/dd2419_ws/src/own_packages/map/') 
import numpy as np
from numpy.core.fromnumeric import shape

import rospy
import tf2_ros
import tf2_geometry_msgs
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, PoseStamped, Transform
import tf.transformations 

def marker_callback(msg):
    for marker in msg.markers:
        #rospy.loginfo(marker)
        broadcast_odom(marker)
        #trans_to_map(marker)

def poseToMatrix(tran, quat):
    T_t = tf.transformations.translation_matrix([tran.x, tran.y, tran.z])
    T_r = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    
    return T_t + T_r

def matrixToPose(T):
    tran = tf.transformations.translation_from_matrix(T)
    quat = tf.transformations.quaternion_from_matrix(T)
    
    return tran, quat 

def mMapMatrix(m, argv=sys.argv):
    path = str(sys.path[-1]) + str(argv[1]) + ".world.json"
    with open(path, 'rb') as f:
        world = json.load(f)  

    markers = world['markers']
    for marker in markers:
        if marker['id'] == m.id:
            trans = tf.transformations.translation_matrix(marker['pose']['position'])
            rot = marker['pose']['orientation']
            rot = tf.transformations.euler_matrix(rot[0], rot[1], rot[2])

    return trans + rot

def broadcast_odom(m):
    #t = m.pose.pose.position 
    #o = m.pose.pose.orientation
    #T = tf.transformations.quaternion_matrix([o.x, o.y, o.z, o.w])
    #T[0][3], T[1][3], T[2][3] = t.x, t.y, t.z
    #T_inv = tf.transformations.inverse_matrix(T)

    c_T_l = poseToMatrix(m.pose.pose.position, m.pose.pose.orientation)
    
    t = tf_buf.lookup_transform('cf1/base_link', 'cf1/camera_link', m.header.stamp)
    b_T_c  = poseToMatrix(t.transform.translation, t.transform.rotation)
    
    b_T_l = np.matmul(b_T_c, c_T_l)
    l_T_b = tf.transformations.inverse_matrix(b_T_l)

    m_T_l = mMapMatrix(m)

    m_T_b = np.matmul(m_T_l, l_T_b)

    if not tf_buf.can_transform('cf1/base_link', 'cf1/odom', m.header.stamp, timeout = rospy.Duration(0.1)):
        return
    t = tf_buf.lookup_transform('cf1/base_link', 'cf1/odom', m.header.stamp, timeout=rospy.Duration(0.1))
    
    b_T_o = poseToMatrix(t.transform.translation, t.transform.rotation)

    m_T_o = np.matmul(m_T_b, b_T_o)

    trans, quat = matrixToPose(m_T_o)

    t = TransformStamped()
    t.header.stamp = m.header.stamp
    t.header.frame_id = 'map'
    t.child_frame_id = 'cf1/odom'
    t.transform.translation.x = trans[0]
    t.transform.translation.y = trans[1]
    t.transform.translation.z = trans[2]

    norm = np.sqrt(quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2)
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1

    broadcaster.sendTransform(t)

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

rospy.init_node('marker_detection')
sub_marker = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)

tf_buf = tf2_ros.Buffer() 
tf_lstn = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()


if __name__ == "__main__":
    rospy.spin()