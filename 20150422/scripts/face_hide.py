#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image # This imports sensor_msgs/Image
from geometry_msgs.msg import Twist # This imports geometry_msgs/Twist
import numpy as np
from cv_bridge import CvBridge

import cv2 # this imports opencv python interface

cascade_path = "/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_alt2.xml"
cascade_eye_path = "/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_eye.xml"

class FaceDetectorMonoNode(object):
    """
This class has feature to subscribe "image" and enbig people face,
then publishes those positions as geometry_msgs/Twist message.
"""

    def __init__(self):
        # make instance of object for image processing
        try:
            self.bridge = CvBridge()
            self.cascade = cv2.CascadeClassifier(cascade_path)
            self.eye_cascade = cv2.CascadeClassifier(cascade_eye_path)
        except Exception as e:
            rospy.logerr("Error: %s" % e)

        # 4. subscribe "image" topic whose message type is sensor_msgs/Image
        self.image_subscriber = rospy.Subscriber("image", Image, self.image_callback)

        # declare publishers
        self.debug_image_publisher = rospy.Publisher("debug_image", Image)

    # 5. define callback function for image topic
    def image_callback(self, msg):
        img_size = (msg.width, msg.height)

        # 6. convert ROS sensor_msgs/Image to OpenCV format
        img_mat = self.bridge.imgmsg_to_cv2(msg)

        # 7. convert image to grey image
        img_gray = cv2.cvtColor(img_mat, cv2.cv.CV_BGR2GRAY)

        # 7. detect face in an image
        face_rects = self.cascade.detectMultiScale(img_gray,
                                                  scaleFactor=1.1,
                                                  minNeighbors=1,
                                                  minSize=(1,1))
        for rect in face_rects:
            roi=img_mat[rect[1]:(rect[1]+rect[3]), rect[0]:(rect[0]+rect[2])]
            roi_gray=img_gray[rect[1]:(rect[1]+rect[3]), rect[0]:(rect[0]+rect[2])]
            # resized_img = cv2.resize(roi, (rect[3]*2, rect[2]*2))
            # roi = resized_img[0:rect[3], 0:rect[2]]

            eyes = self.eye_cascade.detectMultiScale(roi_gray)
            if len(eyes) >= 2:
                mask = np.zeros((img_mat.shape[0], img_mat.shape[1], 1), np.uint8)
                mask[0:img_mat.shape[0], 0:img_mat.shape[1]] = 255
                mask[rect[1]+eyes[0][1]:(rect[1]+eyes[1][1]+eyes[1][3]), rect[0]+eyes[0][0]:(rect[0]+eyes[1][0]+eyes[1][2])] = 0
                img_mat = cv2.bitwise_and(img_mat, img_mat, mask=mask)
            print eyes
            # for (ex,ey,ew,eh) in eyes:
            #     cv2.rectangle(roi,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
            #     eye_roi=roi[ey:(ey+eh),ex:(ex+ew)]
            #     mask = np.zeros((rect[3], rect[2], 1), np.uint8)
            #     mask[0:rect[3], 0:rect[2]] = 0
            #     mask[ey:(ey+eh), ex:(ex+ew)] = 255
            #     roi = cv2.bitwise_and(roi, roi, mask=mask)
            #     # resized_img = cv2.resize(eye_roi, (ew*2, ey*2))
            #     # eye_roi = resized_img[0:eh, 0:ew]
            #     # eye_roi[:,:,:]=0
        pub_debug_img_msg = self.bridge.cv2_to_imgmsg(img_mat, encoding="bgr8")
        self.debug_image_publisher.publish(pub_debug_img_msg)

if __name__ == '__main__':
    rospy.init_node("face_big")
    face_detector = FaceDetectorMonoNode()

    # wait for message comming
    rospy.spin()
