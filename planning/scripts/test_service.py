#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cf_msgs.srv import DronePath, DronePathRequest, DronePathResponse 

def add_two_ints_client():
    rospy.wait_for_service('drone_path')
    try:
        path_service = rospy.ServiceProxy('drone_path', DronePath)
        start = PoseStamped()
        start.header.seq = 0
        start.header.stamp = rospy.Time.now()
        start.header.frame_id = 'map'
        start.pose.position.x, start.pose.position.y, start.pose.position.z = 0.5, 0.5, 0
        start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w = 0, 0, 0, 1

        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 1.5, 0.5, 0
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = 0, 0, 0, 1

        path = path_service(start, goal)
        return path.path 
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.init_node('test_service')
    pub = rospy.Publisher('/path_pub', Path, queue_size=10)
    path = add_two_ints_client()
    

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(path)
        #print(path)
        rate.sleep()
    
    