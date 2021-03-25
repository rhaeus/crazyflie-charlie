#!/usr/bin/env python

from numpy.core.fromnumeric import _cumsum_dispatcher
import rospy 
import numpy as np 


from std_msgs.msg import Bool
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from cf_msgs.srv import DronePath
from nav_msgs.msg import Path

import tf2_ros
import tf2_geometry_msgs 


def flagcallback(msg):
    global is_localized
    is_localized = msg.data

def pose_callback(msg):
    global pose 
    
    if not tf_buf.can_transform('map', msg.header.frame_id, msg.header.stamp, rospy.Duration(0.2)):
        return 
    
    pose = tf_buf.transform(msg, 'map')
    



rospy.init_node('sm2')

global is_localized, pose 
is_localized = False
pose = None 


tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

sub_flag = rospy.Subscriber("/is_localized", Bool, flagcallback)
sub_pose = rospy.Subscriber("/cf1/pose", PoseStamped, pose_callback)

pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

pub_path = rospy.Publisher('/path_pub', Path, queue_size=10)

rospy.wait_for_service('drone_path')
path_service = rospy.ServiceProxy('drone_path', DronePath)



def main():
    global is_localized, pose 

    state = 0
    #print(state)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #print(state)
        if state == 0:
            if is_localized:
                #print('localized')
                state = 1
        

        if state == 1:
            start, goal = PoseStamped(), PoseStamped()
            start = pose 
            
            goal.header.frame_id = 'map'
            goal.pose.position.x = 1.5
            goal.pose.position.y = 3.5
            goal.pose.position.z = 0
            (goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w) = quaternion_from_euler(0,0,0)
            
            path = path_service(start, goal)
            path = path.path 
            if len(path.poses) > 1:
                state = 2 
                #rospy.sleep(0.1)

        if state == 2:
            
            poses = path.poses
            print('--------------------------------')
            print(len(poses))
            print('--------------------------------')
            cmd = Position()
            cmds = []
            for pos in poses:
                pos.header.stamp = rospy.Time.now()
                if not tf_buf.can_transform('map', 'cf1/odom', pos.header.stamp, rospy.Duration(0.8)):
                    print("no transform")
                    break

                pos = tf_buf.transform(pos, 'cf1/odom')
                cmd.header.frame_id = 'cf1/odom'
                cmd.x = pos.pose.position.x
                cmd.y = pos.pose.position.y
                cmd.z = pos.pose.position.z
                _, _, cmd.yaw = euler_from_quaternion((pos.pose.orientation.x,
                                              pos.pose.orientation.y,
                                              pos.pose.orientation.z,
                                              pos.pose.orientation.w))

                print(cmd)
                print('-------------------------------')
                for i in range(50):
                    pub_cmd.publish(cmd)
                    pub_path.publish(path)
                    rate.sleep()
            state = 3




            #print("lift off")
            #cmd = Position()
            #cmd.x = pose.pose.position.x
            #cmd.y = pose.pose.position.y
            #cmd.z = 0.4

            #_, _, cmd.yaw = euler_from_quaternion((pose.pose.orientation.x,
            #                                  pose.pose.orientation.y,
            #                                  pose.pose.orientation.z,
            #                                  pose.pose.orientation.w))
            #pub_cmd.publish(cmd)
            #rate.sleep()



if __name__ == "__main__":
    main()    
