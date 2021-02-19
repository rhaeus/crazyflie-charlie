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

from dd2419_detector_baseline.image_processor import ImageProcessor

class ProcessCameraImage:

  def __init__(self):
    # get parameters from parameter server
    model_path = rospy.get_param('~model_path')
    ann_path = rospy.get_param('~annotation_path')

    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)
    self.image_pub = rospy.Publisher("/cf1/sign_detection/image", Image, queue_size=2)

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
    bbs = self.image_processor.detect_and_classify(pil_image, 0.5)

    pil_image = self.image_processor.overlay_image(pil_image, bbs[0])

    #
    # use numpy to convert the pil_image into a numpy array
    numpy_image=np.array(pil_image)  

    # convert to a openCV2 image, notice the COLOR_RGB2BGR which means that 
    # the color is converted from RGB to BGR format
    opencv_image = cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR) 

   

    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(opencv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('process_camera_image', anonymous=True)

  proc = ProcessCameraImage()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)