#!/usr/bin/env python



from numpy.lib.function_base import angle
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf.transformations 




def callback(m):

    t = TransformStamped()
    t.header.stamp = m.header.stamp
    t.header.frame_id = 'cf1/odom'
    t.child_frame_id = 'cf1/base_footprint'
    t.transform.translation.x = m.pose.position.x
    t.transform.translation.y = m.pose.position.y
    t.transform.translation.z = 0
    
    angles = tf.transformations.euler_from_quaternion([m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w])

    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) =tf.transformations.quaternion_from_euler(0,0,angles[2])
    
    
    broadcaster.sendTransform(t)

    t = TransformStamped()
    t.header.stamp = m.header.stamp
    t.header.frame_id = 'cf1/base_footprint'
    t.child_frame_id = 'cf1/base_stabelized'
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = m.pose.position.z
    


    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) =tf.transformations.quaternion_from_euler(0,0,0)


    broadcaster.sendTransform(t)

    t = TransformStamped()
    t.header.stamp = m.header.stamp
    t.header.frame_id = 'cf1/base_stabelized'
    t.child_frame_id = 'cf1/base_link'
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
     

    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) =tf.transformations.quaternion_from_euler(angles[0],angles[1],0)


    broadcaster.sendTransform(t)



rospy.init_node('base_link_pub')
sub_marker = rospy.Subscriber('/cf1/pose', PoseStamped, callback)

tf_buf = tf2_ros.Buffer() 
tf_lstn = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()



if __name__ == "__main__":
    rospy.spin()
