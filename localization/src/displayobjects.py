#!/usr/bin/env python

import sys
import math
import json

import rospy
import tf2_ros 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3

def transform_from_object(o):
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'object/' + str(o['sign'])
    t.transform.translation = Vector3(*o['pose']['position'])
    roll, pitch, yaw = o['pose']['orientation']
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

    # Load world JSON
    with open(args[1], 'rb') as f:
        world = json.load(f)
    #print(world['roadsigns'])

    # Create a transform for each marker
    transforms = [transform_from_object(o) for o in world['roadsigns']]

    # Publish these transforms statically forever
    rospy.init_node('displaymapobjects')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    rospy.spin()

if __name__ == "__main__":
    main()