#!/usr/bin/env python


import rospy 
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import tf.transformations 
import math

def flagcallback(msg):
    global flag
    flag = msg.data


def posecallback(msg):
    global cfpose
    cfpose = msg


def trans2Map(msg):
    transform = tf_buf.lookup_transform('map',msg.header.frame_id,rospy.Time(0),rospy.Duration(1))
    mapPose = tf2_geometry_msgs.do_transform_pose(msg,transform)
    return mapPose

def trans2Odom(msg):
    transform = tf_buf.lookup_transform('cf1/odom',msg.header.frame_id,rospy.Time(0),rospy.Duration(1))
    odomPose = tf2_geometry_msgs.do_transform_pose(msg,transform)
    return odomPose


def getGoal(state):
    msg = PoseStamped
    
    # exrtract goal from poses
    pose = waypoints(state)

    # set upp message
    msg.header.frame_id = 'map'
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    (msg.pose.orientation.x,
     msg.pose.orientation.y,
     msg.pose.orientation.z,
     msg.pose.orientation.w) =tf.transformations.quaternion_from_euler(pose[3],0,0)
    return msg


def diff(pose):
    x = pose.pose.posisiton.x
    y = pose.pose.posisiton.y
    z = pose.pose.posisiton.z
    angles = tf.transformations.euler_from_quaternion(pose.pose.orientation)
    cfx = cfpose.pose.posisiton.x
    cfy = cfpose.pose.posisiton.y
    cfz = cfpose.pose.posisiton.z
    cfangles = tf.transformations.euler_from_quaternion(cfpose.pose.orientation)
    r = math.sqrt((x-cfx)**2+(y-cfy)**2+(z-cfz)**2+(angles[0]-cfangles[0])**2)
    return r


def main():
    rospy.init_node('statemachine')
    global flag, tf_buf, waypoints
    print("Initiating state machine")
    print("Setting up initial parameters")
    # transform buffer
    tf_buf = tf2_ros.Buffer() 
    tf_lstn = tf2_ros.TransformListener(tf_buf)
    
    #localize flag
    flag = False
    sub_flag = rospy.Subscriber("is_localized", Bool, flagcallback)
    
    # cf pose
    sub_cfpose = rospy.Subscriber("cf1/pose", PoseStamped, posecallback)
    
    # command postion publisher 
    pub_pose = rospy.Publisher('/cf1/cmd_position', PoseStamped,queue_size=10)
    
    # sleep for tf to buffer a bit
    rate = rospy.Rate(1)
    rate.sleep()

    # start planning service
    print("Waiting for planning service")
    rospy.wait_for_service('DronePath')
    planning = rospy.ServiceProxy('DronePath',DronePath)

    # goal coordinates in map frame
    #lst=([x,y,z,yaw][x,y,z,yaw][....])
    waypoints = []
    waypoints.append([1,1,0.4,0])
    waypoints.append([1.5,1,0.4,90])    
    waypoints.append([1,1.5,0.4,180])


    state = 0
    # rate for cmd posistion publish
    rate = rospy.Rate(20)
    print("Starting states")
    while flag:

        if state == 0:
            # transform cf pose to map frame, return PoseStamped msg
            start = trans2Map(cfpose)
            # get goal pose in map frame, return PoseStamed msg
            goal = getGoal(state) 
            # call service for list of pose stamped msg between start and goal
            poses = planning(start, goal)
            #walk trough list 
            for pose in poses:
                while r > 0.1: # how close to sub pose drone need to be to continue 
                    pub_pose.publish(pose)
                    rate.sleep()
                    r = diff(pose)
            r = 1
            state = 1
    

        if state == 1: 
            start = trans2Map(cfpose)
            goal = getGoal(state) 
            poses = planning(start, goal)
            for pose in poses:
                while r > 0.1: 
                    pub_pose.publish(pose)
                    rate.sleep()
                    r = diff(pose)
            r = 1
            state = 2


        if state == 2: 
            start = trans2Map(cfpose)
            goal = getGoal(state) 
            poses = planning(start, goal)
            for pose in poses:
                while r > 0.1: 
                    pub_pose.publish(pose)
                    rate.sleep()
                    r = diff(pose)
            r = 1
            #return to start 
            state = 0

    rospy.spin()

if __name__ == "__main__":
    main()

"""
    if tf_buf.can_transform('cf1/odom','map',rospy.Time.now(), timeout=rospy.Duration(0.1)):
        b = True
        print("Localized")
    else:
        b = False
        print("Not localized")

"""