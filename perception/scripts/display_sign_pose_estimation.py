#!/usr/bin/env python

import math

import rospy
import tf2_ros 
import tf2_geometry_msgs

from tf.transformations import quaternion_from_euler, quaternion_matrix
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped

from cf_msgs.msg import SignMarkerArray

def marker_callback(msg):
    for marker in msg.markers:
        # rospy.loginfo(marker)
        send_transform_from_marker(marker)


def send_transform_from_marker(m):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    marker = PoseStamped()
    marker.header = m.header
    # marker.header.stamp = rospy.Time(0)

    marker.pose.position = m.pose.pose.position
    marker.pose.orientation = m.pose.pose.orientation


    t.header.stamp = m.header.stamp
    t.header.frame_id = 'map'
    t.child_frame_id = '/sign/detected' + str(m.id)

    # marker pose is in frame camera_link
    if not tf_buf.can_transform('map', m.header.frame_id, m.header.stamp, rospy.Duration(1)):
        rospy.logwarn('[display_sign_pose_estimation] No transform from %s to map', m.header.frame_id)
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
    pub_pose_map.publish(marker_map)


rospy.init_node('display_sign_pose_estimation')
sub_marker = rospy.Subscriber('/cf1/sign_detection/pose_estimation', SignMarkerArray, marker_callback)

pub_pose_map = rospy.Publisher('/cf1/sign_detection/pose_estimation_map', PoseStamped, queue_size=2)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)


def main():
    rospy.spin()

if __name__ == "__main__":
    main()