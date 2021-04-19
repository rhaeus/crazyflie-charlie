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

from cf_msgs.msg import SignMarkerArray



def safezone_callback(msg):
    global safezone, cnt
    safezone = msg.data
    cnt = 0

def marker_callback(msg):
    global marker, safezone
    if safezone:
        for marker in msg.markers:
            broadcast_odom(marker)
            trans_to_map(marker)

def sign_callback(msg):
    global road_sign, safezone
    if safezone:
        for sign in msg.markers:
            sign.id = signIdConverter(sign.id)
            broadcast_odom(sign)
            trans_to_map_sign(sign)


def trans_to_map_sign(m):
    road_sign = PoseStamped()
    road_sign.header = m.header
    road_sign.pose = m.pose.pose 

    # Check that there is a sufficient TF tree
    if not tf_buf.can_transform('map', road_sign.header.frame_id, road_sign.header.stamp, timeout = rospy.Duration(0.1)):
        return 

    road_sign = tf_buf.transform(road_sign, 'map')

    trans = TransformStamped()
    trans.header.stamp = m.header.stamp 
    trans.header.frame_id = 'map'
    trans.child_frame_id = 'sign/detected/' + str(m.id)
    trans.transform = Transform(translation=road_sign.pose.position, rotation=road_sign.pose.orientation)

    broadcaster.sendTransform(trans)



# Broadcast a transform between map and a detected aruco marker
def trans_to_map(m):
    aruco_marker = PoseStamped()
    aruco_marker.header = m.header
    aruco_marker.pose = m.pose.pose 

    # Check that there is a sufficient TF tree
    if not tf_buf.can_transform('map', aruco_marker.header.frame_id, aruco_marker.header.stamp, timeout = rospy.Duration(0.1)):
        return 

    aruco_marker = tf_buf.transform(aruco_marker, 'map')

    trans = TransformStamped()
    trans.header.stamp = m.header.stamp 
    trans.header.frame_id = 'map'
    trans.child_frame_id = 'aruco/detected' + str(m.id)
    trans.transform = Transform(translation=aruco_marker.pose.position, rotation=aruco_marker.pose.orientation)

    broadcaster.sendTransform(trans)

# loop through the markers in the map and if the ID is correct, add the coordinates to a list.
def read_static_marker(m):
    if (len(str(m.id))) > 2:
        strr = 'roadsigns'
        identification = 'sign'
    else:
        strr = 'markers'
        identification = 'id'

    markers = world[strr]
    Found = False #error check  
    trans = []
    rot = []
    for marker in markers:
        if marker[identification] == m.id:
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

# Compute the euclidian distance between to points
# NOT IN USE RIGHT NOW
def euc_norm(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)**0.5



# Some kind of data association, just with euclidian distatnce
# NOT IN USE RIGHT NOW
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

# A function that takes compares the distance between markers and the static ones and returns the index for the marker with
# shortest distance. 
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
    global t, old, init, filter, cnt 

    marker_pose = PoseStamped()
    marker_pose.header = m.header
    marker_pose.pose = m.pose.pose 

    if not tf_buf.can_transform('cf1/odom', marker_pose.header.frame_id, marker_pose.header.stamp, timeout=rospy.Duration(0.1)):
        print("No transform between odom and camera link ")
        return 

    # Express the pose of the marker in the odom frame
    camera_in_odom = tf_buf.lookup_transform('cf1/odom', marker_pose.header.frame_id, marker_pose.header.stamp, timeout=rospy.Duration(0.1))
    marker_in_odom = tf2_geometry_msgs.do_transform_pose(marker_pose, camera_in_odom)     
    
    # Express the pose of odom in the seen from the detected marker
    trans, quat = marker_in_odom.pose.position, marker_in_odom.pose.orientation
    l_T_o = tf.transformations.inverse_matrix(tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w]))
    l_T_o[0:4, 3] = np.matmul(l_T_o, np.array([-trans.x, -trans.y, -trans.z, 1]))

    # Read markers in the map with matching ID
    map_translation_markers, map_rotation_markers, Found = read_static_marker(m)
    #print(map_rotation_markers)   

    # If we somehow read a marker that is not in the map (the json file), throw an error
    if Found == False:
        print("Aruco id not in map")
        return

    # If the marker we see is a unique one (typically when we start up), use that one
    # Here it might be good with some additional conditional statement to check if a measurement is way of 
    if len(map_translation_markers) == 1:
        map_translation_marker = map_translation_markers[0]
        map_rotation_marker = map_rotation_markers[0]

    # If the list is longer than one, decide what marker we are looking at
    # This won't trigger the before we have localized at first, since there is no transform between odom and map then 
    else:
        if not tf_buf.can_transform('map', marker_pose.header.frame_id, marker_pose.header.stamp, timeout=rospy.Duration(0.2)):
            return
        
        # Express the pose of the detected marker in the map frame to compare it with the static ones 
        camera_in_map = tf_buf.lookup_transform('map', marker_pose.header.frame_id, marker_pose.header.stamp, timeout=rospy.Duration(0.2))
        marker_in_map = tf2_geometry_msgs.do_transform_pose(marker_pose, camera_in_map)
        marker_in_map_position = np.array([marker_in_map.pose.position.x, marker_in_map.pose.position.y, marker_in_map.pose.position.z])
        marker_in_map_orientation = np.array([marker_in_map.pose.orientation.x, marker_in_map.pose.orientation.y, marker_in_map.pose.orientation.z, marker_in_map.pose.orientation.w])
        min_idx = data_assoc2(map_translation_markers, map_rotation_markers, marker_in_map_position, marker_in_map_orientation)
        print('min index:', map_translation_markers[min_idx])
        #min_idx = data_assoc(map_translation_markers, marker_in_map.pose.position)
        # Continue with the min index 
        map_translation_marker = map_translation_markers[min_idx]
        map_rotation_marker = map_rotation_markers[min_idx]

    # Express the marker/landmark in the map frame, with a transformation matrix
    m_T_l = tf.transformations.quaternion_matrix([map_rotation_marker[0], map_rotation_marker[1], map_rotation_marker[2], map_rotation_marker[3]])
    #m_T_l = tf.transformations.euler_matrix(map_rotation_marker[0] * math.pi/180, map_rotation_marker[1] * math.pi/180, map_rotation_marker[2] * math.pi/180)
    m_T_l[0, 3], m_T_l[1, 3], m_T_l[2, 3] = map_translation_marker[0], map_translation_marker[1], map_translation_marker[2]

    # Use matrix multiplication to obtain the position of the odom frame in the map frame 
    m_T_o = np.matmul(m_T_l,l_T_o)
    angles = tf.transformations.euler_from_matrix(m_T_o)

    # create a transform object between map and odom that only transforms x, y and yaw 
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

    # make the transform smoother
    if init:
        filter = Kalman(xcord = [m_T_o[0,3],m_T_o[1,3],angles[2]])
        init = False
    else:
        filter.predict()
        filter.Kgain()
        t = filter.update(t)
    
    old = t
    broadcaster.sendTransform(t)
    
    # Flag that tells us when we are localized
    msg = Bool()
    if cnt > 10 and tf_buf.can_transform('cf1/odom','map',m.header.stamp, timeout=rospy.Duration(0.1)):
        msg.data = True    
    else:
        msg.data = False
        cnt += 1
    pub.publish(msg)



rospy.init_node('marker_detection')
global broadcaster, pub, init, safezone, cnt

init = True
cnt = 0 
marker = None
road_sign = None
safezone = True   
t = None
tf_buf = tf2_ros.Buffer() 
tf_lstn = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()
pub = rospy.Publisher('/is_localized', Bool, queue_size=10)
sub_safezone = rospy.Subscriber('/cf1/safe_zone', Bool, safezone_callback)
sub_marker = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)
sub_roadsign = rospy.Subscriber("/cf1/sign_detection/pose_estimation", SignMarkerArray, sign_callback)

def main(argv=sys.argv):
    global world


    path = str(argv[1])
    with open(path, 'rb') as f:
        world = json.load(f)  

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Keep on publishing the transform from last time we saw a marker
        if t and rospy.Time.now()-old.header.stamp > rospy.Duration(secs=0.15):
            #print("Using old aruco marker for localization.")
            t.header.stamp = rospy.Time.now()
            broadcaster.sendTransform(t)
        rate.sleep()

if __name__ == "__main__":
    main()

