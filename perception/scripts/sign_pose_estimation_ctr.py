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

        self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)

        self.result_sub = rospy.Subscriber("/cf1/sign_detection/result", DetectionResult, self.callback, queue_size = 1, buff_size=2**24)
        self.pose_pub = rospy.Publisher("/cf1/sign_detection/pose_estimation", SignMarkerArray, queue_size = 10)

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
        info_file = file_path = os.path.join(self.reference_sign_path, 'sign_dimensions.json')

        with open(info_file, 'rb') as json_file:
            ref_info = json.load(json_file)
            for c in ref_info['sign_info']:
                self.ref_dict[int(c['id'])] = c
                if c['id'] in [0,4,5,8,9]:
                    self.ref_dict[int(c['id'])]['shape']='round'
                elif c['id'] in [1,10] :
                    self.ref_dict[int(c['id'])]['shape']='rectangle'
                elif c['id'] == 14 :
                    self.ref_dict[int(c['id'])]['shape']='hex'
                else:
                    self.ref_dict[int(c['id'])]['shape']='triangle'


        for key, info in self.ref_dict.items():
            file_name = info['name'] + ".jpg"
            file_path = os.path.join(self.reference_sign_path, file_name)

            image = cv.imread(file_path)
            height, width, channels = image.shape 

            boundaries = [([17, 15, 50], [90, 80, 255])]
            for (lower, upper) in boundaries:
	            # create NumPy arrays from the boundaries
	            lower = np.array(lower, dtype = "uint8")
	            upper = np.array(upper, dtype = "uint8")
	            # find the colors within the specified boundaries and apply
	            # the mask
	            mask = cv.inRange(image, lower, upper)
	            image = cv.bitwise_and(image, image, mask = mask)
            contours,hier = self.get_contours(image)
            #print(hier, 'hier')
            if len(contours)>0 and info['shape']=='round':
                cont = np.vstack(ctr for ctr in contours)
                # for i, ctr in enumerate(cont) :
                #     cv.drawContours(image, cont, i, (0, 255, 0), 3)
                (x,y),radius = cv.minEnclosingCircle(cont)
                center = (int(x),int(y))
                radius = int(radius)
                image = cv.circle(image,center,radius,(0,255,0),2)
                # ellipse = cv.fitEllipse(cont)
                # image = cv.ellipse(image,ellipse,(0,255,0),2)
            # for i, ctr in enumerate(contours) :
            #     #if hier[0][i][1]!=-1:
            #     cv.drawContours(image, contours, i, (0, 255, 0), 3)
            if len(contours)>0 and info['shape']=='triangle':
                cont = np.vstack(ctr for ctr in contours)
                hull = cv.convexHull(cont)

            info["width"] = width
            info["height"] = height
            info["image"] = image

            real_width = info["real_width"]
            real_height = info["real_height"]

            if info["shape"]=='round':
                x = x * (real_width/width)
                y = y * (real_height/height)
                radius = radius * (real_height/height)
                info['object_points'] = np.array([[x, y + radius,0.0],
                                    [x+radius, y, 0.0],
                                    [x, y-radius, 0.0],
                                    [x-radius, y, 0.0]], dtype=np.float64)
            elif info["shape"]=='triangle':
                leftmost = tuple(hull[hull[:,:,0].argmin()][0])
                rightmost = tuple(hull[hull[:,:,0].argmax()][0])
                topmost = tuple(hull[hull[:,:,1].argmin()][0])
                #middle = (leftmost[0]+rightmost[0]/2, leftmost[1]+rightmost[1]/2)

                leftx = leftmost[0]*(real_width/width)
                lefty = leftmost[1]*(real_height/height)
                rightx = rightmost[0]*(real_width/width)
                righty = rightmost[1]*(real_height/height)
                topx = topmost[0]*(real_width/width)
                topy = topmost[1]*(real_height/height)
                middlex = (leftx + rightx)/2
                middley = (lefty + righty)/2

                object_points = np.array([[leftx,lefty, 0], [topx,topy, 0], [rightx,righty, 0], [middlex, middley, 0]], dtype=np.float64)
                info['object_points']= object_points

            else :
                info["object_points"] = np.array([[-real_width/2.0, real_height/2.0, 0], 
                                    [real_width/2.0, real_height/2.0, 0], 
                                    [real_width/2.0, -real_height/2.0, 0],
                                    [-real_width/2.0, -real_height/2.0, 0]],dtype=np.float64)


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

    def get_contours(self, image):
        imgray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(imgray, 100, 255, 0)
        #60
        
        # roi = image[(bb["x"]):(bb["x"]+bb["width"]),(bb["y"]):(bb["y"] + bb["height"])]
        # black = np.zeros((image.shape[0], image.shape[1], 3), np.uint8)
        # black1 = cv.rectangle(black,(bb['x'], bb['y']),((bb['x']+bb["width"]),(bb['y']+bb['height'])),(255, 255, 255), -1)   #---the dimension of the ROI
        # gray = cv.cvtColor(black,cv.COLOR_BGR2GRAY)               #---converting to gray
        # ret,b_mask = cv.threshold(gray,127,255, 0)

        # fin = cv.bitwise_and(roi,roi,mask = b_mask)


        _, ctr, hier = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        #print('hier', hier)
        #print('contour', ctr)

        return (ctr,hier)

    def callback(self, detection_result):
        image = detection_result.image

        # Convert the image from ROS to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        bbs = convert_msgs.sign_label_msg_array_to_dict_list(detection_result.labels)
        #imgray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        #perfect for sitting charlie
        #boundaries = [([17, 15, 50], [70, 80, 255])]


        #color boundaries we are looking for
        boundaries = [([17, 15, 40], [90, 90, 255])]
        for (lower, upper) in boundaries:
	        # create NumPy arrays from the boundaries
	        lower = np.array(lower, dtype = "uint8")
	        upper = np.array(upper, dtype = "uint8")
	        # find the colors within the specified boundaries and apply
	        # the mask
	        mask = cv.inRange(cv_image, lower, upper)
	        output = cv.bitwise_and(cv_image, cv_image, mask = mask)
        #imgray = cv.cvtColor(output, cv.COLOR_BGR2GRAY)
        #ret, thresh = cv.threshold(imgray, 90, 255, 0)

        sign_marker_array = SignMarkerArray()
        sign_marker_array.header = image.header
        sign_marker_array.header.frame_id = 'cf1/camera_link'

        #if no bbx detected, the image is black
        black = np.zeros((cv_image.shape[0], cv_image.shape[1], 3), np.uint8)
        gray = cv.cvtColor(black,cv.COLOR_BGR2GRAY)
        ret,b_mask = cv.threshold(gray,127,255, 0)
        output_n = cv.bitwise_and(output, output,mask = b_mask)

        
        for bb in bbs:
            # match features with reference image
            cat = bb["category"]
            ref = self.ref_dict[cat]


            # calculate object points and image points

            object_points = ref["object_points"]
            image_points = [[]]


            # compute mask to only look within bbox
            maski = self.compute_mask(output, bb)
            output_n = cv.bitwise_and(output, output, mask = maski)

            #imgray = cv.cvtColor(output, cv.COLOR_BGR2GRAY)
            #ret, thresh = cv.threshold(imgray, 90, 255, 0)
        
        #get contours
            contours,hier = self.get_contours(output_n)

            
            #cv.drawContours(output_n, contours, -1, (0,255,0),3)

            if len(contours)>0 and self.ref_dict[int(bb['category'])]['shape']=='triangle':
                cont = np.vstack(ctr for ctr in contours)
                hull = cv.convexHull(cont)
                leftmost = tuple(hull[hull[:,:,0].argmin()][0])
                #print(leftmost, 'leftmost')
                rightmost = tuple(hull[hull[:,:,0].argmax()][0])
                topmost = tuple(hull[hull[:,:,1].argmin()][0])
                middle = (leftmost[0]+rightmost[0]/2, leftmost[1]+rightmost[1]/2)

                image_points = np.array([leftmost, topmost, rightmost, middle], dtype=np.float64)
                
                #cv.drawContours(output_n, cont, 0, (0, 255, 0), 3)
                

            if len(contours)>0 and self.ref_dict[int(bb['category'])]['shape']=='round':
                cont = np.vstack(ctr for ctr in contours)
                #ellipse fitting
                # ellipse = cv.fitEllipse(cont)
                # #print(ellipse)
                # output_n = cv.ellipse(output_n,ellipse,(0,255,0),2)

                #circle fitting
                (x,y),radius = cv.minEnclosingCircle(cont)
                center = (int(x),int(y))
                radius = int(radius)
                if radius>20:
                    output_n = cv.circle(output_n,center,radius,(0,255,0),2)

        #for i, ctr in enumerate(contours) :
            #if hier[0][i][-1]==-1:
                #structuringElement = cv.getStructuringElement(cv.MORPH_ELLIPSE, (40, 40))
                #cv.morphologyEx(output_n, output_n, cv.MORPH_CLOSE, structuringElement)

                #ellipse = cv.fitEllipse(ctr)
                #output_n = cv.ellipse(output_n,ellipse,(0,255,0),2)
                #cv.drawContours(output_n, contours, i, (0, 255, 0), 3)

                

                    top = (x, y + radius)
                    bottom = (x, y - radius)

                    right = (x + radius, y)
                    left = (x - radius, y)
                    #print(bb['category'])
                    print(left, 'left')

                    image_points = np.array([top, right, bottom, left],dtype=np.float64)

            # else :

            #     top_left_corner = (bb["x"], bb["y"])
            #     top_right_corner = (bb["x"] + bb["width"], bb["y"])

            #     bottom_right_corner = (bb["x"] + bb["width"], bb["y"] + bb["height"])
            #     bottom_left_corner = (bb["x"], bb["y"] + bb["height"])


            #     image_points = np.array([[top_left_corner], [top_right_corner], [bottom_right_corner], [bottom_left_corner]],dtype=np.float64)
            
            # Publish the image
            try:
            #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(closing, "passthrough"))
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_n, "8UC3"))
            except CvBridgeError as e:
                print(e) 
            print("obj", object_points)
            print('im', image_points)
            if image_points != [[]]:
                # use cv2.solvePnP for pose estimation
                (_, rotation_vector, translation_vector) = cv.solvePnP(
                    object_points[:4], image_points[:4], self.camera_matrix, self.distortion)

                # create pose
                p = Pose()

                p.position.x = translation_vector[0]
                print(translation_vector[0])
                p.position.y = translation_vector[1]
                p.position.z = translation_vector[2]

                (p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w) = quaternion_from_euler(math.radians(-90.0), 0.0, math.radians(-90.0))
                #p.orientation.w) = quaternion_from_euler(math.radians(rotation_vector[0]),
                #                                            math.radians(rotation_vector[1]),
                #                                            math.radians(rotation_vector[2]))

                # create SignMarker 
                sign_marker = SignMarker()
                sign_marker.header = image.header
                sign_marker.header.frame_id = 'cf1/camera_link'
                sign_marker.id = bb["category"]

                sign_marker.pose.pose = p
                # sign_marker.pose.covariance = 

                # add SignMarker to SignMarkerArray
                sign_marker_array.markers.append(sign_marker)


        # publish SignMArkerArray
        self.pose_pub.publish(sign_marker_array)

    

def main(args):
  rospy.init_node('sign_pose_estimation_ctr', anonymous=True)

  proc = SignPoseEstimation()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)