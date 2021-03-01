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

        self.image_pub = rospy.Publisher("/cf1/sign_detection/feature_matches", Image, queue_size=2)

        self.camera_info_sub = rospy.Subscriber("/cf1/camera/camera_info", CameraInfo, self.camera_info_callback)

        self.distortion = []
        self.camera_matrix = []

        self.bridge = CvBridge()

        # calculate features for all signs
        self.load_reference_features()

    def camera_info_callback(self, data):
        self.distortion = data.D
        m = np.array(data.K)
        self.camera_matrix = np.reshape(m, (3,3))

    def load_reference_features(self):
        # reads info in res/reference_signs/info.json
        # reads images in res/reference_signs and calculates features and descriptors
        # stores 
        # - width 
        # - height
        # - category id = key in dict = name of image
        # - features 
        # - descriptors

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
            kp, des = self.compute_feature_descriptors(image)
            info["kp"] = kp
            info["des"] = des
            info["image"] = image
            # info["object_points"] = self.keypoints_to_object_points(kp, info)


    def compute_mask(self, image, bbx):
        height, width, channels = image.shape 
        # mask = np.zeros_like(image)
        mask = np.zeros((height, width), dtype=np.uint8)

        bbx_top_left_x = bbx["x"]
        bbx_top_left_y = bbx["y"]
        bbx_width = bbx["width"]
        bbx_height = bbx["height"]

        # print("y:", (bbx_top_left_y, bbx_top_left_y + bbx_height))
        # print("x:", (bbx_top_left_x, bbx_top_left_x + bbx_width))

        for y in range(bbx_top_left_y, bbx_top_left_y + bbx_height):
            for x in range(bbx_top_left_x, bbx_top_left_x + bbx_width):
                if x < width and y < height:
                    mask[y, x] = 255
        return mask

    def compute_feature_descriptors(self, image, mask=None):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        sift = cv.xfeatures2d.SIFT_create()
        # find the keypoints and descriptors with SIFT
        kp, des = sift.detectAndCompute(gray, mask)

        return (kp, des)

    def keypoints_to_object_points(self, kps, info):
        object_points = []
        real_width = float(info["real_width"])
        real_height = float(info["real_height"])
        image_width = float(info["width"])
        image_height = float(info["height"])

        for point in kps:
            px = point.pt[0]
            py = point.pt[1]

            ox = px/image_width*real_width - real_width/2.0
            oy = py/image_height*real_height - real_height/2.0
            oz = 0

            object_points.append((ox, oy, oz))

        return np.array(object_points)

    def keypoints_to_image_points(self, kps):
        image_points = []

        for point in kps:
            px = point.pt[0]
            py = point.pt[1]

            image_points.append((px, py))

        return np.array(image_points)


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
            # compute mask
            mask = self.compute_mask(cv_image, bb)
            # detect features in camera image
            kp, des = self.compute_feature_descriptors(cv_image, mask)

            # match features with reference image
            cat = bb["category"]
            ref = self.ref_dict[cat]

            # create BFMatcher object
            bf = cv.BFMatcher(cv.NORM_L2, crossCheck=True)
            # Match descriptors.
            matches = bf.match(des, ref["des"])
            # Sort them in the order of their distance.
            matches = sorted(matches, key = lambda x:x.distance)

            # Draw first 10 matches.
            img3 = cv.drawMatches(cv_image,kp,ref["image"],ref["kp"],matches[:5],None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
             # Publish the image
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(img3, "bgr8"))
            except CvBridgeError as e:
                print(e)

            # calculate object points and image points

            # we need at least 4 matches
            if len(matches) < 4:
                rospy.loginfo("too few matches")
                return

            object_keypoints = []
            image_keypoints = []

            # get 4 best matches
            for i in range(4):
                match = matches[i]
                object_keypoints.append(ref["kp"][match.trainIdx])
                image_keypoints.append(kp[match.queryIdx])
                

            image_points = self.keypoints_to_image_points(image_keypoints)
            object_points = self.keypoints_to_object_points(object_keypoints, ref)

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
  rospy.init_node('sign_pose_estimation', anonymous=True)

  proc = SignPoseEstimation()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)