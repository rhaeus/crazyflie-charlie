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
import tf2_ros 
import tf2_geometry_msgs

import convert_msgs

from cf_msgs.msg import DetectionResult, SignMarker, SignMarkerArray

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped, Vector3

class SignPoseEstimation:

    def __init__(self):
        global world
        self.ann_path = rospy.get_param('~annotation_path')
        self.reference_sign_path = rospy.get_param('~reference_sign_path')

        self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)

        self.result_sub = rospy.Subscriber("/cf1/sign_detection/result", DetectionResult, self.callback, queue_size = 1, buff_size=2**24)
        self.pose_pub = rospy.Publisher("/cf1/sign_detection/pose_estimation", SignMarkerArray, queue_size = 10)

        # self.image_pub = rospy.Publisher("/cf1/sign_detection/feature_matches", Image, queue_size=2)

        self.camera_info_sub = rospy.Subscriber("/cf1/camera/camera_info", CameraInfo, self.camera_info_callback)

        self.distortion = []
        self.camera_matrix = []

        self.bridge = CvBridge()

        self.tf_buf   = tf2_ros.Buffer()
        self.tf_lstn  = tf2_ros.TransformListener(self.tf_buf)

        self.world = world

        # calculate features for all signs
        self.load_reference_info()

    def camera_info_callback(self, data):
        self.distortion = data.D
        m = np.array(data.K)
        self.camera_matrix = np.reshape(m, (3,3))

    def get_category_dict(self, ann_file):
        category_dict = {}
        with open(ann_file, 'rb') as json_file:
            data = json.load(json_file)
            for c in data['categories']:
                category_dict[int(c['id'])] = c

        # print(category_dict)
        return category_dict

    def compute_feature_descriptors(self, image, mask=None):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        _, thresh = cv.threshold(gray, 60, 255, 0)

        sift = cv.xfeatures2d.SIFT_create()
        # find the keypoints and descriptors with SIFT
        kp, des = sift.detectAndCompute(thresh, mask)

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

            #ox = px/image_width*real_width - real_width/2.0
            #oy = py/image_height*real_height - real_height/2.0
            ox = px * (real_width/image_width)
            oy = py * (real_height/image_height)
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

            boundaries = [([0, 0, 0], [200, 200, 255])]
            for (lower, upper) in boundaries:
	            # create NumPy arrays from the boundaries
	            lower = np.array(lower, dtype = "uint8")
	            upper = np.array(upper, dtype = "uint8")
	            # find the colors within the specified boundaries and apply
	            # the mask
	            mask = cv.inRange(image, lower, upper)
	            image = cv.bitwise_and(image, image, mask = mask)
            contours,hier, _ = self.get_contours(image)

            #Integrating feature detection
            kp, des = self.compute_feature_descriptors(image)
            info["kp"] = kp
            info["des"] = des
            
            #print(hier, 'hier')
            if len(contours)>0 and info['shape']=='round':
                cont = np.vstack(ctr for ctr in contours)
                # for i, ctr in enumerate(cont) :
                #     cv.drawContours(image, cont, i, (0, 255, 0), 3)

                #circular
                # (x,y),radius = cv.minEnclosingCircle(cont)
                # center = (int(x),int(y))
                # radius = int(radius)
                # image = cv.circle(image,center,radius,(0,255,0),2)

                #rectangular
                x,y,w,h = cv.boundingRect(cont)
                image = cv.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)

                # ellipse = cv.fitEllipse(cont)
                # image = cv.ellipse(image,ellipse,(0,255,0),2)
            # for i, ctr in enumerate(contours) :
            #     #if hier[0][i][1]!=-1:
            #     cv.drawContours(image, contours, i, (0, 255, 0), 3)
            elif len(contours)>0 and info['shape']=='triangle':
                cont = np.vstack(ctr for ctr in contours)

                #circular
                # (x,y),radius = cv.minEnclosingCircle(cont)
                # center = (int(x),int(y))
                # radius = int(radius)
                # image = cv.circle(image,center,radius,(0,255,0),2)
                #print(cont)

                #triangular --not good--
                #hull = cv.convexHull(cont)
                #cv.drawContours(image,cont, -1, (0, 255, 0),3)
                #hull = cv.convexHull(cont)

                #rectangular
                x,y,w,h = cv.boundingRect(cont)
                image = cv.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)

            elif len(contours)>0 and info['shape']=='rectangle':
                cont = np.vstack(ctr for ctr in contours)
                #cv.drawContours(image,cont, -1, (0, 255, 0),3)

                #rectangular
                x,y,w,h = cv.boundingRect(cont)
                image = cv.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)

            elif len(contours)>0 and info['shape']=='hex':
                cont = np.vstack(ctr for ctr in contours)

                #rectangular
                x,y,w,h = cv.boundingRect(cont)
                image = cv.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)

            info["width"] = width
            info["height"] = height

            real_width = info["real_width"]
            real_height = info["real_height"]

            if len(contours)>0 and info["shape"]=='round':

                #circular
                # x = x * (real_width/width)
                # y = y * (real_height/height)
                # radius = radius * (real_height/height)
                # info['object_points'] = np.array([[x, y + radius,0.0],
                #                     [x+radius, y, 0.0],
                #                     [x, y-radius, 0.0],
                #                     [x-radius, y, 0.0]], dtype=np.float64)

                #rectangular
                x = x * (real_width/width)
                y = y * (real_height/height)
                w = w * (real_width/width)
                h = h * (real_height/height)

                top = (x + w/2, y)
                bottom = (x + w/2, y + h)

                right = (x, y + h/2)
                left = (x + w, y + h/2)

                # top_l = (x,y)
                # top_r = (x + w,y)
                # bottom_l = (x, y+h)
                # bottom_r = (x+w, y+h)
                info['object_points'] = np.array([[top[0], top[1],0.0],[right[0], right[1], 0.0],[bottom[0], bottom[1], 0.0],[left[0], left[1], 0.0]], dtype=np.float64)
                #info['object_points'] = np.array([[top_r[0], top_r[1],0.0],[top[0], top[1],0.0],[top_l[0], top_l[1], 0.0],[left[0], left[1],0.0],[bottom_l[0], bottom_l[1], 0.0],[bottom[0], bottom[1],0.0],[bottom_r[0], bottom_r[1], 0.0],[right[0], right[1],0.0]], dtype=np.float64)

            elif len(contours)>0 and info["shape"]=='triangle':
                # leftmost = tuple(cont[cont[:,:,0].argmin()][0])
                # rightmost = tuple(cont[cont[:,:,0].argmax()][0])
                # topmost = tuple(cont[cont[:,:,1].argmin()][0])
                # #middle = (leftmost[0]+rightmost[0]/2, leftmost[1]+rightmost[1]/2)

                # leftx = leftmost[0]*(real_width/width)
                # lefty = leftmost[1]*(real_height/height)
                # rightx = rightmost[0]*(real_width/width)
                # righty = rightmost[1]*(real_height/height)
                # topx = topmost[0]*(real_width/width)
                # topy = topmost[1]*(real_height/height)
                # middlex = (leftx + rightx)/2
                # middley = (lefty + righty)/2

                # object_points = np.array([[leftx,lefty, 0], [topx,topy, 0], [rightx,righty, 0], [middlex, middley, 0]], dtype=np.float64)
                # info['object_points']= object_points

                # triangle_points = np.array([[[leftmost[0],leftmost[1]]], [[topmost[0],topmost[1]]], [[rightmost[0],rightmost[1]]], [[middlex*(width/real_width),middley*(height/real_height)]]], dtype=np.int32)
                # cv.drawContours(image,triangle_points, -1, (0, 255, 255),3)
                # print(object_points, info['name'])

                #circular
                # x = x * (real_width/width)
                # y = y * (real_height/height)
                # radius = radius * (real_height/height)
                # info['object_points'] = np.array([[x, y + radius,0.0],
                #                     [x+radius, y, 0.0],
                #                     [x, y-radius, 0.0],
                #                     [x-radius, y, 0.0]], dtype=np.float64)

                #rectangular
                x = x * (real_width/width)
                y = y * (real_height/height)
                w = w * (real_width/width)
                h = h * (real_height/height)

                top = (x + w/2, y)
                bottom = (x + w/2, y + h)

                right = (x, y + h/2)
                left = (x + w, y + h/2)
                info['object_points'] = np.array([[top[0], top[1],0.0],[right[0], right[1], 0.0],[bottom[0], bottom[1], 0.0],[left[0], left[1], 0.0]], dtype=np.float64)

            elif len(contours)>0 and info["shape"]=='rectangle':

                #rectangular
                x = x * (real_width/width)
                y = y * (real_height/height)
                w = w * (real_width/width)
                h = h * (real_height/height)

                top = (x + w/2, y)
                bottom = (x + w/2, y + h)

                right = (x, y + h/2)
                left = (x + w, y + h/2)
                info['object_points'] = np.array([[top[0], top[1],0.0],[right[0], right[1], 0.0],[bottom[0], bottom[1], 0.0],[left[0], left[1], 0.0]], dtype=np.float64)

            elif len(contours)>0 and info["shape"]=='hex':

                #rectangular
                x = x * (real_width/width)
                y = y * (real_height/height)
                w = w * (real_width/width)
                h = h * (real_height/height)

                top = (x + w/2, y)
                bottom = (x + w/2, y + h)

                right = (x, y + h/2)
                left = (x + w, y + h/2)
                info['object_points'] = np.array([[top[0], top[1],0.0],[right[0], right[1], 0.0],[bottom[0], bottom[1], 0.0],[left[0], left[1], 0.0]], dtype=np.float64)

            else :
                info["object_points"] = np.array([[-real_width/2.0, real_height/2.0, 0], 
                                    [real_width/2.0, real_height/2.0, 0], 
                                    [real_width/2.0, -real_height/2.0, 0],
                                    [-real_width/2.0, -real_height/2.0, 0]],dtype=np.float64)

            
            info["image"] = image


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
        ret, thresh = cv.threshold(imgray, 60, 255, 0)
        
        # roi = image[(bb["x"]):(bb["x"]+bb["width"]),(bb["y"]):(bb["y"] + bb["height"])]
        # black = np.zeros((image.shape[0], image.shape[1], 3), np.uint8)
        # black1 = cv.rectangle(black,(bb['x'], bb['y']),((bb['x']+bb["width"]),(bb['y']+bb['height'])),(255, 255, 255), -1)   #---the dimension of the ROI
        # gray = cv.cvtColor(black,cv.COLOR_BGR2GRAY)               #---converting to gray
        # ret,b_mask = cv.threshold(gray,127,255, 0)

        # fin = cv.bitwise_and(roi,roi,mask = b_mask)


        _, ctr, hier = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        #print('hier', hier)
        #print('contour', ctr)

        return (ctr,hier, thresh)

    def callback(self, detection_result):
        category_dict = self.get_category_dict(self.ann_path)
        
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
        boundaries = [([20, 20, 30], [100, 100, 255])]
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

        #print('bbs', bbs)
        masks = []
        for bb in bbs:
            #print(bb['category'])
            # match features with reference image
            cat = bb["category"]
            ref = self.ref_dict[cat]

            #if self.ref_dict[int(bb['category'])]['shape']=='triangle':
                # compute mask to only look within bbox
            maski = self.compute_mask(output, bb)
                #masks.append(maski)
                #for i in range(len(masks)):
            output_n = cv.bitwise_and(output, output, mask = maski)

            #imgray = cv.cvtColor(output, cv.COLOR_BGR2GRAY)
            #ret, thresh = cv.threshold(imgray, 90, 255, 0)


            ##################FEATURE DETECTION METHOD##############################################################
            # detect features in camera image
            kp, des = self.compute_feature_descriptors(output_n, maski)

            # create BFMatcher object
            bf = cv.BFMatcher(cv.NORM_L2, crossCheck=True)
            # Match descriptors.
            matches = bf.match(des, ref["des"])
            # Sort them in the order of their distance.
            matches = sorted(matches, key = lambda x:x.distance)

            # Draw first 10 matches.
            img3 = cv.drawMatches(output_n,kp,ref["image"],ref["kp"],matches[:10],(0, 255, 0),flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

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
                
            image_points_sift = self.keypoints_to_image_points(image_keypoints)
            object_points_sift = self.keypoints_to_object_points(object_keypoints, ref)

            #initial guess
            initial_guess = PoseStamped()
            initial_guess.header = image.header
            initial_guess.header.frame_id = 'map'

            sign_id = bb['category']
            sign_name = category_dict[sign_id]['name']
                
            map_position=[0,0,0]
            map_orient = [0,0,0]
            for i in self.world['roadsigns']:
                if i['sign']==sign_name:
                    map_position = Vector3(*i['pose']['position'])
                    roll, pitch, yaw = i['pose']['orientation']

            
            initial_guess.pose.position.x = map_position.x
            initial_guess.pose.position.y = map_position.y
            initial_guess.pose.position.z = map_position.z
            (initial_guess.pose.orientation.x,
            initial_guess.pose.orientation.y,
            initial_guess.pose.orientation.z,
            initial_guess.pose.orientation.w) = quaternion_from_euler(math.radians(roll),
                                                            math.radians(pitch),
                                                            math.radians(yaw))

            if not self.tf_buf.can_transform('cf1/camera_link', initial_guess.header.frame_id, initial_guess.header.stamp, rospy.Duration(1)):
                rospy.logwarn('[display_sign_pose_estimation] initial_guess No transform from %s to cf1/camera_link', initial_guess.header.frame_id)
                return

            initial_guess_camera = self.tf_buf.transform(initial_guess, 'cf1/camera_link')

            guess_t = np.array([initial_guess_camera.pose.position.x, initial_guess_camera.pose.position.y, initial_guess_camera.pose.position.z])
            (roll, pitch, yaw) = euler_from_quaternion((initial_guess_camera.pose.orientation.x, initial_guess_camera.pose.orientation.y, initial_guess_camera.pose.orientation.z, initial_guess_camera.pose.orientation.w))
            guess_r = np.array([roll, pitch, yaw])






            #use cv2.solvePnP for pose estimation
            (_, rotation_vector_sift, translation_vector_sift) = cv.solvePnP(
               object_points_sift[:], image_points_sift[:], self.camera_matrix, self.distortion, guess_r, guess_t, True, cv.SOLVEPNP_ITERATIVE)

            # use cv2.solvePnP for pose estimation
            # (_, rotation_vector_sift, translation_vector_sift) = cv.solvePnP(
            #         object_points_sift[:4], image_points_sift[:4], self.camera_matrix, self.distortion)





            ##########CONTOUR BOUNDING BOX METHOD#################################

            # calculate object points and image points

            object_points = ref["object_points"]
            image_points = [[]]
        
            #get contours
            contours,hier, _ = self.get_contours(output_n)
            #print('contours', contours)

            
            #cv.drawContours(output_n, contours, -1, (0,255,0),3)

            if len(contours)>0 and self.ref_dict[int(bb['category'])]['shape']=='triangle':
                cont = np.vstack(ctr for ctr in contours)
                cv.drawContours(output_n, cont, -1, (0, 255, 255), 3)

                #circle fitting
                # (x,y),radius = cv.minEnclosingCircle(cont)
                # center = (int(x),int(y))
                # radius = int(radius)

                
                # if radius>20:
                #     output_n = cv.circle(output_n,center,radius,(0,255,0),2)

                #     top = (x, y + radius)
                #     bottom = (x, y - radius)

                #     right = (x + radius, y)
                #     left = (x - radius, y)
                #     #print(bb['category'])

                #rectangle fitting
                x,y,w,h = cv.boundingRect(cont)
                output_n = cv.rectangle(output_n,(x,y),(x+w,y+h),(0,255,255),2)

                if h>20:
                    top = (x + w/2, y)
                    bottom = (x + w/2, y + h)

                    right = (x, y + h/2)
                    left = (x + w, y + h/2)
                    

                    image_points = np.array([top, right, bottom, left],dtype=np.float64)


                #triangular fitting --not good--
                # cont = np.vstack(ctr for ctr in contours)
                # epsilon = 0.3*cv.arcLength(cont,True)
                # approx = cv.approxPolyDP(cont,epsilon,True)
                # hull = cv.convexHull(cont)
                # leftmost = tuple(cont[cont[:,:,0].argmin()][0])
                # #print(leftmost, 'leftmost')
                # rightmost = tuple(cont[cont[:,:,0].argmax()][0])
                # topmost = tuple(cont[cont[:,:,1].argmin()][0])
                # middle = ((leftmost[0]+rightmost[0])/2, (leftmost[1]+rightmost[1])/2)

                # image_points = np.array([leftmost, topmost, rightmost, middle], dtype=np.float64)
                # print(image_points)
                # #hull = cv.convexHull(image_points)
                # triangle_cont = np.array([[[leftmost[0], leftmost[1]]],[[topmost[0], topmost[1]]],[[rightmost[0], rightmost[1]]]], dtype=np.int32)
                
                # cv.drawContours(output_n, triangle_cont, -1, (0, 255, 255), 3)



                # # # Publish the reference image
                # try:
                # #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(closing, "passthrough"))
                #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.ref_dict[int(bb['category'])]['image'], "8UC3"))
                # except CvBridgeError as e:
                #     print(e)
                






            if len(contours)>0 and self.ref_dict[int(bb['category'])]['shape']=='round':
                cont = np.vstack(ctr for ctr in contours)
                cv.drawContours(output_n, cont, -1, (0, 255, 0), 3)

                #ellipse fitting
                # ellipse = cv.fitEllipse(cont)
                # print(ellipse)
                # output_n = cv.ellipse(output_n,ellipse,(0,255,0),2)

                #rectangular fitting
                x,y,w,h = cv.boundingRect(cont)
                output_n = cv.rectangle(output_n,(x,y),(x+w,y+h),(0,255,255),2)

                #circle fitting
                # (x,y),radius = cv.minEnclosingCircle(cont)
                # center = (int(x),int(y))
                # radius = int(radius)

                
                #circular
                #if radius>20:
                #     output_n = cv.circle(output_n,center,radius,(0,255,0),2)
                    # top = (x, y + radius)
                    # bottom = (x, y - radius)

                    # right = (x + radius, y)
                    # left = (x - radius, y)
                    # #print(bb['category'])

                #rectangular
                if h>20:
                    top = (x + w/2, y)
                    bottom = (x + w/2, y + h)

                    right = (x, y + h/2)
                    left = (x + w, y + h/2)

                    # top_r = (x + w, y)
                    # top_l = (x, y)
                    # bottom_l = (x, y+h)
                    # bottom_r = (x+w, y+h)
                    

                    image_points = np.array([top, right, bottom, left],dtype=np.float64)
                    #image_points = np.array([top_r, top, top_l, left, bottom_l, bottom, bottom_r, right],dtype=np.float64)
                    #image_points = np.array([top_l, top_r, bottom_r, bottom_l],dtype=np.float64)



                # # Publish the reference image
                # try:
                # #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(closing, "passthrough"))
                #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.ref_dict[int(bb['category'])]['image'], "8UC3"))
                # except CvBridgeError as e:
                #     print(e)

            if len(contours)>0 and self.ref_dict[int(bb['category'])]['shape']=='rectangle':
                cont = np.vstack(ctr for ctr in contours)
                cv.drawContours(output_n, cont, -1, (0, 255, 0), 3)

                #rectangular fitting
                x,y,w,h = cv.boundingRect(cont)
                output_n = cv.rectangle(output_n,(x,y),(x+w,y+h),(0,255,255),2)

                #rectangular
                if h>20:
                    top = (x + w/2, y)
                    bottom = (x + w/2, y + h)

                    right = (x, y + h/2)
                    left = (x + w, y + h/2)
                    

                    image_points = np.array([top, right, bottom, left],dtype=np.float64)

                # # Publish the reference image
                # try:
                # #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(closing, "passthrough"))
                #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.ref_dict[int(bb['category'])]['image'], "8UC3"))
                # except CvBridgeError as e:
                #     print(e)


            if len(contours)>0 and self.ref_dict[int(bb['category'])]['shape']=='hex':
                cont = np.vstack(ctr for ctr in contours)
                cv.drawContours(output_n, cont, -1, (0, 255, 0), 3)

                #rectangular fitting
                x,y,w,h = cv.boundingRect(cont)
                output_n = cv.rectangle(output_n,(x,y),(x+w,y+h),(0,255,255),2)

                #rectangular
                if h>20:
                    top = (x + w/2, y)
                    bottom = (x + w/2, y + h)

                    right = (x, y + h/2)
                    left = (x + w, y + h/2)
                    

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

            #print("obj", object_points)
            #print('im', image_points)
            if image_points != [[]]:

                #print(self.world['roadsigns'][0]['pose'])
                #initial guess
                # initial_guess = PoseStamped()
                # initial_guess.header = image.header
                # initial_guess.header.frame_id = 'map'

                # sign_id = bb['category']
                # sign_name = category_dict[sign_id]['name']
                
                # map_position=[0,0,0]
                # map_orient = [0,0,0]
                # for i in self.world['roadsigns']:
                #     if i['sign']==sign_name:
                #         map_position = Vector3(*i['pose']['position'])
                #         roll, pitch, yaw = i['pose']['orientation']

            
                # initial_guess.pose.position.x = map_position.x
                # initial_guess.pose.position.y = map_position.y
                # initial_guess.pose.position.z = map_position.z
                # (initial_guess.pose.orientation.x,
                # initial_guess.pose.orientation.y,
                # initial_guess.pose.orientation.z,
                # initial_guess.pose.orientation.w) = quaternion_from_euler(math.radians(roll),
                #                                                 math.radians(pitch),
                #                                                 math.radians(yaw))

                # if not self.tf_buf.can_transform('cf1/camera_link', initial_guess.header.frame_id, initial_guess.header.stamp, rospy.Duration(1)):
                #     rospy.logwarn('[display_sign_pose_estimation] initial_guess No transform from %s to cf1/camera_link', initial_guess.header.frame_id)
                #     return

                # initial_guess_camera = self.tf_buf.transform(initial_guess, 'cf1/camera_link')

                # guess_t = np.array([initial_guess_camera.pose.position.x, initial_guess_camera.pose.position.y, initial_guess_camera.pose.position.z])
                # (roll, pitch, yaw) = euler_from_quaternion((initial_guess_camera.pose.orientation.x, initial_guess_camera.pose.orientation.y, initial_guess_camera.pose.orientation.z, initial_guess_camera.pose.orientation.w))
                # guess_r = np.array([roll, pitch, yaw])






                # use cv2.solvePnP for pose estimation
                #(_, rotation_vector, translation_vector) = cv.solvePnP(
                #    object_points[:], image_points[:], self.camera_matrix, self.distortion, guess_r, guess_t, True, cv.SOLVEPNP_ITERATIVE)
                (_, rotation_vector, translation_vector) = cv.solvePnP(
                    object_points[:4], image_points[:4], self.camera_matrix, self.distortion)
                #(_, rotation_vector, translation_vector) = cv.solvePnP(
                #    object_points[:4], image_points[:4], self.camera_matrix, self.distortion, True, cv.SOLVEPNP_IPPE_SQUARE)


            



                # create pose
                p = Pose()

                p.position.x = translation_vector[0]
                #print(translation_vector[0])
                p.position.y = translation_vector[1]
                p.position.z = translation_vector[2]

                (p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w) = quaternion_from_euler(math.radians(rotation_vector_sift[0]-90),
                                                           math.radians(rotation_vector_sift[1]),
                                                           math.radians(rotation_vector_sift[2]-90))

                #p.orientation.w) = quaternion_from_euler(math.radians(-90.0), 0.0, math.radians(-90.0))

                # create SignMarker 
                sign_marker = SignMarker()
                sign_marker.header = image.header
                sign_marker.header.frame_id = 'cf1/camera_link'
                sign_marker.id = bb["category"]

                sign_marker.pose.pose = p
                
                # sign_marker.pose.covariance = 

                # add SignMarker to SignMarkerArray
                sign_marker_array.markers.append(sign_marker)

        # try:
        #     #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(closing, "passthrough"))
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_n, "8UC3"))
        # except CvBridgeError as e:
        #     print(e) 

        # publish SignMArkerArray
        self.pose_pub.publish(sign_marker_array)

    

def main(argv=sys.argv):
    global world
    # Let ROS filter through the arguments
    args = rospy.myargv(argv=argv)    
    # Load world JSON
    #print(args[1])
    with open(args[1]) as f:
        world = json.load(f)

    rospy.init_node('sign_pose_estimation_ctr', anonymous=True)

    proc = SignPoseEstimation()

    print("running...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv.destroyAllWindows()

if __name__ == '__main__':
    main()