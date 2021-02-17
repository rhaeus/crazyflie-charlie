#!/usr/bin/env python
 
import rospy
import tf2_ros
import tf2_geometry_msgs
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, PoseStamped, Transform

def marker_callback(msg):
    for marker in msg.markers:
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

rospy.init_node('marker_detection')
sub_marker = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)

tf_buf = tf2_ros.Buffer() 
tf_lstn = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()

if __name__ == "__main__":
    rospy.spin()