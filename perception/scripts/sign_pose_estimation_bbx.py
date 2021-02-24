#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import os
import json
import math

import convert_msgs

from cf_msgs.msg import DetectionResult, SignMarker, SignMarkerArray

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose

class SignPoseEstimation:

    def __init__(self):
        self.reference_sign_path = rospy.get_param('~reference_sign_path')

        self.result_sub = rospy.Subscriber("/cf1/sign_detection/result", DetectionResult, self.callback)
        self.pose_pub = rospy.Publisher("/cf1/sign_detection/pose_estimation", SignMarkerArray, queue_size=2)

        # self.image_pub = rospy.Publisher("/cf1/sign_detection/feature_matches", Image, queue_size=2)

        self.camera_info_sub = rospy.Subscriber("/cf1/camera/camera_info", CameraInfo, self.camera_info_callback)

        self.distortion = []
        self.camera_matrix = []

        self.bridge = CvBridge()

        # calculate features for all signs
        self.load_reference_info()

    def camera_info_callback(self, data):
        self.distortion = data.D
        m = np.array(data.K)
        self.camera_matrix = np.reshape(m, (3,3))

    def load_reference_info(self):
        # reads info in res/reference_signs/info.json
        # stores 
        # corners of the signs in object coordinates

        self.ref_dict = {}
        info_file = file_path = os.path.join(self.reference_sign_path, 'info.json')

        with open(info_file, 'rb') as json_file:
            ref_info = json.load(json_file)
            for c in ref_info['sign_info']:
                self.ref_dict[int(c['id'])] = c


        for key, info in self.ref_dict.items():
            file_name = info['name'] + ".jpg"
            file_path = os.path.join(self.reference_sign_path, file_name)

            image = cv.imread(file_path)
            height, width, channels = image.shape 

            info["width"] = width
            info["height"] = height
            info["image"] = image

            real_width = info["real_width"]
            real_height = info["real_height"]

            info["object_points"] = np.array([[-real_width/2.0, real_height/2.0, 0], 
                                    [real_width/2.0, real_height/2.0, 0], 
                                    [real_width/2.0, -real_height/2.0, 0],
                                    [-real_width/2.0, -real_height/2.0, 0]],dtype=np.float64)


    def callback(self, detection_result):
        image = detection_result.image

        # Convert the image from ROS to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        bbs = convert_msgs.sign_label_msg_array_to_dict_list(detection_result.labels)


        sign_marker_array = SignMarkerArray()
        sign_marker_array.header = image.header
        sign_marker_array.header.frame_id = 'cf1/camera_link'

        for bb in bbs:
            # match features with reference image
            cat = bb["category"]
            ref = self.ref_dict[cat]


            # calculate object points and image points

            object_points = ref["object_points"]

            top_left_corner = (bb["x"], bb["y"])
            top_right_corner = (bb["x"] + bb["width"], bb["y"])

            bottom_right_corner = (bb["x"] + bb["width"], bb["y"] + bb["height"])
            bottom_left_corner = (bb["x"], bb["y"] + bb["height"])


            image_points = np.array([[top_left_corner], [top_right_corner], [bottom_right_corner], [bottom_left_corner]],dtype=np.float64)
          

            # use cv2.solvePnP for pose estimation
            (_, rotation_vector, translation_vector) = cv.solvePnP(
                object_points[:4], image_points[:4], self.camera_matrix, self.distortion)

            # create pose
            p = Pose()

            p.position.x = translation_vector[0]
            p.position.y = translation_vector[1]
            p.position.z = translation_vector[2]

            (p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w) = quaternion_from_euler(math.radians(rotation_vector[0]),
                                                            math.radians(rotation_vector[1]),
                                                            math.radians(rotation_vector[2]))

            # create SignMarker 
            sign_marker = SignMarker()
            sign_marker.header = image.header
            sign_marker.id = bb["category"]

            sign_marker.pose.pose = p
            # sign_marker.pose.covariance = 

            # add SignMarker to SignMarkerArray
            sign_marker_array.markers.append(sign_marker)


        # publish SignMArkerArray
        self.pose_pub.publish(sign_marker_array)

    

def main(args):
  rospy.init_node('sign_pose_estimation_bbx', anonymous=True)

  proc = SignPoseEstimation()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)