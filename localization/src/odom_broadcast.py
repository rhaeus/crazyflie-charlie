#!/usr/bin/env python

import sys 
import json 
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
        #broadcast_odom(msg.markers[0])

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

    return T 

"""
OLD CODE 
def delRollPitch(T):
    rot = list(tf.transformations.euler_from_matrix(T))
    rot[1] = 0
    rot[2] = 0
    T_old = T
    T = tf.transformations.euler_matrix(rot[0], rot[1], rot[2])
    T[0,3]= T_old[0,3]
    T[1,3]= T_old[1,3]
    T[2,3]= T_old[2,3]    
    print(T)
    return T 

def delZ(T):
    T[2,3] = 0
    return T 
"""    

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
    t.transform.translation.z = trans[2]

    norm = np.sqrt(quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2)
    t.transform.rotation.x = quat[0]/norm
    t.transform.rotation.y = quat[1]/norm
    t.transform.rotation.z = quat[2]/norm
    t.transform.rotation.w = quat[3]/norm

    broadcaster.sendTransform(t)
    rospy.sleep(0.1)

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
    global tf_buf, broadcaster, world

    path = str(argv[1])
    with open(path, 'rb') as f:
        world = json.load(f)  


    sub_marker = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)

    tf_buf = tf2_ros.Buffer() 
    tf_lstn = tf2_ros.TransformListener(tf_buf)
    broadcaster = tf2_ros.TransformBroadcaster()
    rospy.spin()

if __name__ == "__main__":
    main()
