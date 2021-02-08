#!/usr/bin/env python

import sys
import math
import json

import rospy
import tf2_ros 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3

def transform_from_marker(m):
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'aruco/marker' + str(m['id'])
    t.transform.translation = Vector3(*m['pose']['position'])
    roll, pitch, yaw = m['pose']['orientation']
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))
    return t

def main(argv=sys.argv):
    # Let ROS filter through the arguments
    args = rospy.myargv(argv=argv)

    rospy.init_node('displaymapmarkers')

    # Load world JSON
    with open(args[1], 'rb') as f:
        world = json.load(f)

    # Create a transform for each marker
    transforms = [transform_from_marker(m) for m in world['markers']]

    t = TransformStamped()

    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "map"
    # t.child_frame_id = 'aruco/marker2132332425346'
    # t.transform.translation.x = 0.01
    # t.transform.translation.y = 0
    # t.transform.translation.z = 0.02
    # t.transform.rotation.x = 0
    # t.transform.rotation.y = 0
    # t.transform.rotation.z = 0
    # t.transform.rotation.w = 0

    # Publish these transforms statically forever
    
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    # broadcaster.sendTransform(t)
    rospy.spin()

if __name__ == "__main__":
    main()