#!/usr/bin/env python

import rospy

from cf_msgs.msg import DetectionResult
from std_msgs.msg import Bool

global intruder_detected
intruder_detected = False

global intruder_id

def callback(detection_result):
    for label in detection_result.labels:
        if label.category_id == intruder_id:
            intruder_detected = True


if __name__ == '__main__':
    rospy.init_node('intruder_detector', anonymous=True)
    intruder_id = rospy.get_param('~intruder_id', 0)

    detection_result_sub = rospy.Subscriber("/cf1/sign_detection/result", DetectionResult, callback)
    pub = rospy.Publisher('/cf1/intruder_detection_result', Bool, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(intruder_detected)
        rate.sleep()

    