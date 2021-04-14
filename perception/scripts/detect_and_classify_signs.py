#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PIL_Image

from cf_msgs.msg import DetectionResult

from dd2419_detector_baseline.image_processor import ImageProcessor

import convert_msgs

class DetectAndClassifySigns:

  def __init__(self):
    # get parameters from parameter server
    model_path = rospy.get_param('~model_path')
    ann_path = rospy.get_param('~annotation_path')

    self.bbx_dist_thres = rospy.get_param('~bbx_dist_thres', 100)
    self.confidence_thres = rospy.get_param('~confidence_thres', 0.5)

    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback, queue_size = 1, buff_size=2**24)
    self.result_pub = rospy.Publisher("/cf1/sign_detection/result", DetectionResult, queue_size = 10)

    self.bridge = CvBridge()
    self.image_processor = ImageProcessor(model_path, ann_path)

  def callback(self,data):
    # Convert the image from ROS to OpenCV format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # TODO: better way to get image to pytorch?

    # convert from openCV2 to PIL. Notice the COLOR_BGR2RGB which means that 
    # the color is converted from BGR to RGB
    color_coverted = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    pil_image = PIL_Image.fromarray(color_coverted)

    # use detector to detect and classify
    bbs = self.image_processor.detect_and_classify(pil_image, self.confidence_thres)

    # filter multiple responses per sign
    bbs = self.image_processor.reduce_bbx_nb(bbs, self.bbx_dist_thres)

    
    labels = []

    for bb in bbs[0]:
        labels.append(convert_msgs.dict_entry_to_sign_label_msg(bb))

    result = DetectionResult()
    result.image = data
    result.labels = labels

    # Publish the result
    self.result_pub.publish(result)


def main(args):
  rospy.init_node('detect_and_classify_signs', anonymous=True)

  proc = DetectAndClassifySigns()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)