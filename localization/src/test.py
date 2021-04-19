#!/usr/bin/env python

import sys 
import json 
import numpy as np
import tf2_geometry_msgs

from kalmanfilter import Kalman

import math
import rospy
import tf2_ros
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, PoseStamped, Transform, Quaternion
from std_msgs.msg import Bool
import tf.transformations 


def marker_callback(msg):
    global marker
    for marker in msg.markers:
        broadcast_odom(marker)
        trans_to_map(marker)


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

def read_static_marker(m):
    markers = world['markers']
    Found = False
    for marker in markers:
        if marker['id'] == m.id:
            trans = marker['pose']['position']
            rot = marker['pose']['orientation']
            Found = True

    return trans, rot, Found

def broadcast_odom(m):
    global t, old, init, filter

    marker_pose = PoseStamped()
    marker_pose.header = m.header
    marker_pose.pose = m.pose.pose 

    if not tf_buf.can_transform('cf1/odom', 'cf1/camera_link', m.header.stamp, timeout=rospy.Duration(0.1)):
        print("No transform between odom and camera link ")
        return 

    camera_in_odom = tf_buf.lookup_transform('cf1/odom', 'cf1/camera_link', m.header.stamp, timeout=rospy.Duration(0.1))
    marker_in_odom = tf2_geometry_msgs.do_transform_pose(marker_pose, camera_in_odom)     


    trans, quat = marker_in_odom.pose.position, marker_in_odom.pose.orientation
    l_T_o = tf.transformations.inverse_matrix(tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w]))
    l_T_o[0:4, 3] = np.matmul(l_T_o, np.array([-trans.x, -trans.y, -trans.z, 1]))

    map_translation_marker, map_rotation_marker, Found = read_static_marker(m)   

    if Found == False:
        print("frame id not in world")
        return

    m_T_l = tf.transformations.euler_matrix(map_rotation_marker[0] * math.pi/180, map_rotation_marker[1] * math.pi/180, map_rotation_marker[2] * math.pi/180)
    m_T_l[0, 3], m_T_l[1, 3], m_T_l[2, 3] = map_translation_marker[0], map_translation_marker[1], map_translation_marker[2]

    m_T_o = np.matmul(m_T_l,l_T_o)
    angles = tf.transformations.euler_from_matrix(m_T_o)

    t = TransformStamped()
    t.header.stamp = m.header.stamp
    t.header.frame_id = 'map'
    t.child_frame_id = 'cf1/odom'
    t.transform.translation.x = m_T_o[0,3] 
    t.transform.translation.y = m_T_o[1,3] 
    t.transform.translation.z = 0
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) =tf.transformations.quaternion_from_euler(0,0,angles[2])


    if init:
        filter = Kalman(xcord = [m_T_o[0,3],m_T_o[1,3],angles[2]])
        init = False
    else:
        filter.predict()
        filter.Kgain()
        t = filter.update(t)
    
    old = t
    broadcaster.sendTransform(t)
    
    msg = Bool()
    if tf_buf.can_transform('cf1/odom','map',m.header.stamp, timeout=rospy.Duration(0.1)):
        msg.data = True    
    else:
        msg.data = False
    pub.publish(msg)



rospy.init_node('marker_detection')
global broadcaster, pub, init

init = True

marker = None 
t = None
sub_marker = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)


tf_buf = tf2_ros.Buffer() 
tf_lstn = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()
pub = rospy.Publisher('/is_localized', Bool, queue_size=10)

def main(argv=sys.argv):
    global world

    path = str(argv[1])
    with open(path, 'rb') as f:
        world = json.load(f)  

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if t and rospy.Time.now()-old.header.stamp > rospy.Duration(secs=0.15):
            #print("Using old aruco marker for localization.")
            t.header.stamp = rospy.Time.now()
            broadcaster.sendTransform(t)
        rate.sleep()

if __name__ == "__main__":
    main()
