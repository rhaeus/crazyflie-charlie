#!/usr/bin/env python

import math

import rospy
import tf2_ros 
import tf2_geometry_msgs

from tf.transformations import quaternion_from_euler, quaternion_matrix
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped

from aruco_msgs.msg import MarkerArray

def marker_callback(msg):
    for marker in msg.markers:
        # rospy.loginfo(marker)
        send_transform_from_marker(marker)


def send_transform_from_marker(m):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    marker = PoseStamped()
    marker.header = m.header
    marker.header.stamp = rospy.Time(0)

    marker.pose.position = m.pose.pose.position
    marker.pose.orientation = m.pose.pose.orientation


    t.header.stamp = m.header.stamp
    t.header.frame_id = 'map'
    t.child_frame_id = '/aruco/detected' + str(m.id)

    # marker pose is in frame camera_link
    if not tf_buf.can_transform('map', m.header.frame_id, rospy.Time(0), rospy.Duration(1)):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map', m.header.frame_id)
        return

    marker_map = tf_buf.transform(marker, 'map')

    t.transform.translation.x = marker_map.pose.position.x
    t.transform.translation.y = marker_map.pose.position.y
    t.transform.translation.z = marker_map.pose.position.z

    t.transform.rotation.x = marker_map.pose.orientation.x
    t.transform.rotation.y = marker_map.pose.orientation.y
    t.transform.rotation.z = marker_map.pose.orientation.z
    t.transform.rotation.w = marker_map.pose.orientation.w

    br.sendTransform(t)


rospy.init_node('marker_transformation')
sub_marker = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)


def main():
    rospy.spin()

if __name__ == "__main__":
    main()