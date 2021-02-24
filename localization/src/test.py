#!/usr/bin/env python

from numpy.lib.financial import rate
import rospy
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import TransformStamped, PoseStamped, Transform
from aruco_msgs.msg import MarkerArray


"""
Sudo code:
1. Read aruco posistion 
2. Calc cam-cf-odom position in map frame
3. Update map to odom transform 
4. 
"""


def msg_callback(msg):
    for marker in msg.markers:
        aruco2odom(marker)


def aruco2odom(marker):

    pt = PoseStamped()
    pt.header = marker.header
    pt.header.frame_id = 'aruco/marker'+ str(marker.id)
    pt.pose.position.x = -1*marker.pose.pose.position.x
    pt.pose.position.y = -1*marker.pose.pose.position.y
    pt.pose.position.x = -1*marker.pose.pose.position.z
    pt.pose.orientation = marker.pose.pose.orientation 
    #pt.pose = marker.pose.pose
    
    if not tf_buf.can_transform('map', pt.header.frame_id, pt.header.stamp, timeout = rospy.Duration(0.1)):
        print('No transform between map and marker')
        return 

    
    # cam pose in map frame
    cfcam_pose = tf_buf.transform(pt, 'map')
    #cfcam_pose = tf_buf.transform(pt, 'base_link')
    
    
    pt = PoseStamped()
    pt.header.stamp = cfcam_pose.header.stamp
    pt.header.frame_id = 'cf1/camera_link'
    pt.pose.position = cfcam_pose.pose.position
    pt.pose.orientation = cfcam_pose.pose.orientation
    

    if not tf_buf.can_transform('cf1/odom', pt.header.frame_id, pt.header.stamp, timeout = rospy.Duration(0.1)):
        print('No transform from camera to odom')
        return 

    odom_pose = tf_buf.transform(pt,'cf1/odom')
    
    trans = TransformStamped()
    trans.header.stamp = marker.header.stamp 
    trans.header.frame_id = 'map'
    trans.child_frame_id = 'cf1/odom'
    trans.transform = Transform(translation=odom_pose.pose.position, rotation=odom_pose.pose.orientation)

    broadcaster.sendTransform(trans)


rospy.init_node('test')
rospy.Subscriber('/aruco/markers', MarkerArray, msg_callback)

tf_buf = tf2_ros.Buffer() 
tf_lstn = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()

rospy.loginfo('Running test')

def main():

    rospy.spin()



if __name__ == "__main__":
    main()
