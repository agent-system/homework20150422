#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

# 1. import rospy to enable ROS feature of python
import rospy

# 2. import message files for python
# cf. you can find what contains in a message by executing `rosmsg show <msg>`
# e.g. rosmsg show sensor_msgs/Image
from sensor_msgs.msg import Image # This imports sensor_msgs/Image

# Tips: This is special utility for converting between ROS Image message and OpenCV Image format.
from cv_bridge import CvBridge

import cv2 # this imports opencv python interface

cascade_path = "/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_alt2.xml"

class ImageProcessorMonoNode(object):
    """
This class has feature to subscribe "image" and calculate feature points.
"""

    def __init__(self):
        # make instance of object for image processing
        try:
            self.bridge = CvBridge()
            self.cascade = cv2.CascadeClassifier(cascade_path)
        except Exception as e:
            rospy.logerr("Error: %s" % e)

        # 4. subscribe "image" topic whose message type is sensor_msgs/Image
        self.image_subscriber = rospy.Subscriber("image", Image, self.image_callback)

        # declare publishers
        self.debug_image_publisher = rospy.Publisher("debug_image", Image)
        self.detector = cv2.FeatureDetector_create("ORB")

    # 5. define callback function for image topic
    def image_callback(self, msg):
        img_size = (msg.width, msg.height)

        # 6. convert ROS sensor_msgs/Image to OpenCV format
        img_mat = self.bridge.imgmsg_to_cv2(msg)

        # 7. detect face in an image
        keypoints = self.detector.detect(img_mat)

        # 12. publish debug image
        img_kp = cv2.drawKeypoints(img_mat, keypoints, None)
        self.debug_image_publisher.publish(self.bridge.cv2_to_imgmsg(img_kp, encoding="bgr8"))

if __name__ == '__main__':
    # 3. At first, we must set node name, and register to the master.
    # In this case, node name is ip
    rospy.init_node("ip")
    face_detector = ImageProcessorMonoNode()

    # wait for message comming
    rospy.spin()
