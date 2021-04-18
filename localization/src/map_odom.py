#!/usr/bin/env python
from __future__ import print_function

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

# loop through the markers in the map and if the ID is correct, add their coordinates
def read_static_marker(m):
    markers = world['markers']
    Found = False 
    trans = []
    rot = []
    for marker in markers:
        if marker['id'] == m.id:
            #arr = np.array([marker['pose']['position'][0], marker['pose']['position'][1], marker['pose']['position'][2]])
            #print(np.linalg.norm(arr))
            #trans.append(marker['pose']['position'])
            trans.append(np.array([marker['pose']['position'][0], 
                                marker['pose']['position'][1], 
                                marker['pose']['position'][2]]))

            rot.append(tf.transformations.quaternion_from_euler(marker['pose']['orientation'][0]* math.pi/180, 
                                                                marker['pose']['orientation'][1]* math.pi/180, 
                                                                marker['pose']['orientation'][2]* math.pi/180))
            #rot.append(marker['pose']['orientation'])
            Found = True

    return trans, rot, Found

# compute the euclidian distance between to points
def euc_norm(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)**0.5

#def pose_to_list_pose(pose):


def data_assoc(static_markers, marker_position):
    min = 10000
    min_idx = 0
    for idx in range(len(static_markers)):
        norm = euc_norm(static_markers[idx], [marker_position.x, marker_position.y, marker_position.z])
        #print(static_markers[idx], [marker_position.x, marker_position.y, marker_position.z])
        #print(norm)
        #print('-------------------------------')
        if norm <= min:
            min = norm
            min_idx = idx
    

    return min_idx

def data_assoc2(translation_static, rotation_static, translation, rotation):
    translation = translation/float(np.linalg.norm(translation))
    pose = np.concatenate((translation, rotation))/float(np.sqrt(2))
    min = 100
    min_idx = 0
    for idx in range(len(translation_static)):
        temp_translation = translation_static[idx]/float(np.linalg.norm(translation_static[idx]))
        temp_pose = np.concatenate((temp_translation, rotation_static[idx]))/float(np.sqrt(2))
        #print(temp_pose, pose)
        print(translation_static[idx])
        #print('--------------------------------------')
        #bc = np.sqrt(np.dot(pose, temp_pose))
        #d = np.sqrt(1 - bc)
        #print('bc:', bc)
        #print('distance:', d)
        #print('---------')
        d = np.linalg.norm(pose-temp_pose)
        print('distance:', d)
        print('---------')
        if d <= min:
            min = d
            min_idx = idx

    return min_idx


def broadcast_odom(m):
    global t, old, init, filter, means

    marker_pose = PoseStamped()
    marker_pose.header = m.header
    marker_pose.pose = m.pose.pose 

    if not tf_buf.can_transform('cf1/odom', marker_pose.header.frame_id, marker_pose.header.stamp, timeout=rospy.Duration(0.4)):
        print("No transform between odom and camera link ")
        return 

    camera_in_odom = tf_buf.lookup_transform('cf1/odom', marker_pose.header.frame_id, marker_pose.header.stamp, timeout=rospy.Duration(0.1))
    marker_in_odom = tf2_geometry_msgs.do_transform_pose(marker_pose, camera_in_odom)     
    
    trans, quat = marker_in_odom.pose.position, marker_in_odom.pose.orientation
    l_T_o = tf.transformations.inverse_matrix(tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w]))
    l_T_o[0:4, 3] = np.matmul(l_T_o, np.array([-trans.x, -trans.y, -trans.z, 1]))

    map_translation_markers, map_rotation_markers, Found = read_static_marker(m)
    #print(map_rotation_markers)   

    if Found == False:
        print("Aruco id not in map")
        return

    if len(map_translation_markers) == 1:
        map_translation_marker = map_translation_markers[0]
        map_rotation_marker = map_rotation_markers[0]

    else:
        if not tf_buf.can_transform('map', marker_pose.header.frame_id, marker_pose.header.stamp, timeout=rospy.Duration(0.2)):
            return
        camera_in_map = tf_buf.lookup_transform('map', marker_pose.header.frame_id, marker_pose.header.stamp, timeout=rospy.Duration(0.2))
        marker_in_map = tf2_geometry_msgs.do_transform_pose(marker_pose, camera_in_map)
        marker_in_map_position = np.array([marker_in_map.pose.position.x, marker_in_map.pose.position.y, marker_in_map.pose.position.z])
        marker_in_map_orientation = np.array([marker_in_map.pose.orientation.x, marker_in_map.pose.orientation.y, marker_in_map.pose.orientation.z, marker_in_map.pose.orientation.w])
        min_idx = data_assoc2(map_translation_markers, map_rotation_markers, marker_in_map_position, marker_in_map_orientation)
        print('min index:', map_translation_markers[min_idx])
        #min_idx = data_assoc(map_translation_markers, marker_in_map.pose.position)
        map_translation_marker = map_translation_markers[min_idx]
        map_rotation_marker = map_rotation_markers[min_idx]

    m_T_l = tf.transformations.quaternion_matrix([map_rotation_marker[0], map_rotation_marker[1], map_rotation_marker[2], map_rotation_marker[3]])
    #m_T_l = tf.transformations.euler_matrix(map_rotation_marker[0] * math.pi/180, map_rotation_marker[1] * math.pi/180, map_rotation_marker[2] * math.pi/180)
    m_T_l[0, 3], m_T_l[1, 3], m_T_l[2, 3] = map_translation_marker[0], map_translation_marker[1], map_translation_marker[2]

    m_T_o = np.matmul(m_T_l,l_T_o)
    angles = tf.transformations.euler_from_matrix(m_T_o)

    if len(means) == 40:
        means.pop(0)

    means.append((m_T_o[0,3], m_T_o[1,3], angles[2]))
    xm, ym, yawm = 0, 0, 0
    for i in range(len(means)):
        xm+= means[i][0]
        ym+= means[i][1]
        yawm+= means[i][2]
    
    xm = xm/float(len(means))
    ym = ym/float(len(means))
    yawm = yawm/float(len(means))



    t = TransformStamped()
    t.header.stamp = m.header.stamp
    t.header.frame_id = 'map'
    t.child_frame_id = 'cf1/odom'
    t.transform.translation.x = xm #m_T_o[0,3] 
    t.transform.translation.y = ym #m_T_o[1,3] 
    t.transform.translation.z = 0
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) =tf.transformations.quaternion_from_euler(0,0, yawm) #angles[2])


    # if init:
    #     filter = Kalman(xcord = [m_T_o[0,3],m_T_o[1,3],angles[2]])
    #     init = False
    # else:
    #     filter.predict()
    #     filter.Kgain()
    #     t = filter.update(t)
    
    old = t
    broadcaster.sendTransform(t)
    rospy.Rate(10).sleep()
    
    msg = Bool()
    if tf_buf.can_transform('cf1/odom','map',m.header.stamp, timeout=rospy.Duration(0.2)):
        msg.data = True    
    else:
        msg.data = False
    pub.publish(msg)



rospy.init_node('marker_detection')
global broadcaster, pub, init, means

init = True
means = []
marker = None 
t = None
tf_buf = tf2_ros.Buffer() 
tf_lstn = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()
pub = rospy.Publisher('/is_localized', Bool, queue_size=10)
sub_marker = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)

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