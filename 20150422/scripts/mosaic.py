#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

# 1. import rospy to enable ROS feature of python
import rospy

# 2. import message files for python
# cf. you can find what contains in a message by executing `rosmsg show <msg>`
# e.g. rosmsg show sensor_msgs/Image
from sensor_msgs.msg import Image # This imports sensor_msgs/Image
from geometry_msgs.msg import Twist # This imports geometry_msgs/Twist

# Tips: This is special utility for converting between ROS Image message and OpenCV Image format.
from cv_bridge import CvBridge

import cv2 # this imports opencv python interface

cascade_path = "/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_alt2.xml"

class FaceDetectorMonoNode(object):
    """
This class has feature to subscribe "image" and detect people face,
then publishes those positions as geometry_msgs/Twist message.
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
        self.face_pose_publisher = rospy.Publisher("twist", Twist)
        self.debug_image_publisher = rospy.Publisher("debug_image", Image)

    def image_mosaic(self, img):
        w = img.shape[1]
        h = img.shape[0]
        img = cv2.resize(img, (w / 20, h / 20))
        img = cv2.resize(img, (w, h), interpolation = cv2.cv.CV_INTER_NN)
        return img

     # 5. define callback function for image topic
    def image_callback(self, msg):
        img_mat = self.bridge.imgmsg_to_cv2(msg)
        img_mat = self.image_mosaic(img_mat)
        pub_debug_img_msg = self.bridge.cv2_to_imgmsg(img_mat, encoding="bgr8")
        self.debug_image_publisher.publish(pub_debug_img_msg)

if __name__ == '__main__':
    # 3. At first, we must set node name, and register to the master.
    # In this case, node name is face_detector_mono
    rospy.init_node("face_detector_mono")
    face_detector = FaceDetectorMonoNode()

    # wait for message comming
    rospy.spin()
