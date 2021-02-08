#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import math
import numpy as np
import random
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped

import tf2_ros
import tf2_geometry_msgs

class object_detector:

  def __init__(self):
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.image_callback)
    self.camera_info_sub = rospy.Subscriber("/cf1/camera/camera_info", CameraInfo, self.camera_info_callback)

    self.distortion = []
    self.camera_matrix = []


  def camera_info_callback(self, data):
      self.distortion = data.D
      m = np.array(data.K)
      self.camera_matrix = np.reshape(m, (3,3))



  def image_callback(self,data):
    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of the color we look for in the HSV space
    # lower = np.array([0,0,250])
    # upper = np.array([255,5,255])
    lower = np.array([94, 127, 20])
    upper = np.array([126, 255, 200])

    # Threshold the HSV image to get only the pixels in ranage
    mask = cv2.inRange(hsv, lower, upper)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cv_image, cv_image, mask= mask)

    # get gray scale
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    # blur = cv2.GaussianBlur(gray, (3,3), 0)
    thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)[1]

    # check if object is there
    # and obtain the outermost points -> corner points
    object_there = False

    height = gray.shape[0] # y
    width = gray.shape[1] # x

    top_left = np.array([width, height])
    top_right = np.array([0, height])
    bottom_left = np.array([width,0])
    bottom_right = np.array([0,0])


    for y in range(height):
        # if object_there:
        #     break
        for x in range(width):
            if int(thresh[y,x]) > 0:
                if x+y < top_left[0]+top_left[1]:
                    top_left[0] = x
                    top_left[1] = y
                
                if x+y > bottom_right[0]+bottom_right[1]:
                    bottom_right[0] = x
                    bottom_right[1] = y

                if x+height-y < bottom_left[0]+height-bottom_left[1]:
                    bottom_left[0] = x
                    bottom_left[1] = y

                if width-x+y < width-top_right[0]+top_right[1]:
                    top_right[0] = x
                    top_right[1] = y

                object_there = True
                # break

    if object_there:

        cv2.circle(cv_image, (top_left[0], top_left[1]), 5, (0, 50, 255), -1)
        cv2.circle(cv_image, (bottom_right[0], bottom_right[1]), 5, (0, 255, 255), -1)
        cv2.circle(cv_image, (bottom_left[0], bottom_left[1]), 5, (255, 50, 0), -1)
        cv2.circle(cv_image, (top_right[0], top_right[1]), 5, (255, 255, 0), -1)
    
        print('object detected')
        object_points_in_image = np.array([top_left, top_right, bottom_right, bottom_left],dtype=np.float64)
        print('location in image (corner points): ')
        print(object_points_in_image)

        # dimenions of object
        # model_points = np.array([[-0.1, 0.1, 0], 
        #                             [0.1, 0.1, 0], 
        #                             [0.1, -0.1, 0],
        #                             [-0.1, -0.1, 0]],dtype=np.float64)

        model_points = np.array([[-0.1, 0.0625, 0], 
                                    [0.1, 0.0625, 0], 
                                    [0.1, -0.0625, 0],
                                    [-0.1, -0.0625, 0]],dtype=np.float64)

        # model_points = np.array([[-0.2, 0.125, 0], 
        #                             [0.2, 0.125, 0], 
        #                             [0.2, -0.125, 0],
        #                             [-0.2, -0.125, 0]],dtype=np.float64)

    
        # use solvePnP
        (_, rotation_vector, translation_vector) = cv2.solvePnP(
                model_points, object_points_in_image, self.camera_matrix, self.distortion)

        # print('rotation ',rotation_vector)
        # print('translation ',translation_vector)

        # transform the estimated pose to map frame
        p = PoseStamped()
        p.header.frame_id = 'camera_link'
        p.header.stamp = rospy.Time(0)

        p.pose.position.x = translation_vector[0]
        p.pose.position.y = translation_vector[1]
        p.pose.position.z = translation_vector[2]

        (p.pose.orientation.x,
        p.pose.orientation.y,
        p.pose.orientation.z,
        p.pose.orientation.w) = quaternion_from_euler(math.radians(rotation_vector[0]),
                                                        math.radians(rotation_vector[1]),
                                                        math.radians(rotation_vector[2]))
        # print('posestamped ', p)

        if not tf_buf.can_transform('map', p.header.frame_id, rospy.Time(0), rospy.Duration(1)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to map', p.header.frame_id)
            return

        object_map = tf_buf.transform(p, 'map')

        location = object_map.pose.position
        roll, pitch, yaw = euler_from_quaternion((object_map.pose.orientation.x,
                                                object_map.pose.orientation.y,
                                                object_map.pose.orientation.z,
                                                object_map.pose.orientation.w))

        orientation = np.array([math.degrees(roll), math.degrees(pitch), math.degrees(yaw)])


        print('location in map: ')
        print(location)
        print('orientation in map: ')
        print(orientation)

    # Publish the image
    try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(closing, "passthrough"))
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

rospy.init_node('detect_object', anonymous=True)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def main(args):
  

  od = object_detector()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)