#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Qishen Ha <haqishen@mi.t.u-tokyo.ac.jp>

import rospy

from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge
import cv2

cascade_path = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml"

class FaceDetectorMonoNode(object):

    def __init__(self):
        try:
            self.bridge = CvBridge()
            self.cascade = cv2.CascadeClassifier(cascade_path)
        except Exception as e:
            rospy.logerr("Error: %s" % e)

        self.image_subscriber = rospy.Subscriber("image", Image, self.image_callback)
        self.face_pose_publisher = rospy.Publisher("twist", Twist)
        self.debug_image_publisher = rospy.Publisher("debug_image", Image)

    def image_callback(self, msg):
        img_size = (msg.width, msg.height)

        img_mat = self.bridge.imgmsg_to_cv2(msg)

        img_grey = cv2.cvtColor(img_mat, cv2.cv.CV_BGR2GRAY)

        # args:(color,scale,?,sizelimit?)
        face_rects = self.cascade.detectMultiScale(img_grey,
                                                  scaleFactor=1.1,
                                                  minNeighbors=1,
                                                   minSize=(1,1))

        if len(face_rects) > 0:
            rect = face_rects[0]
            face_rect_origin = (rect[0], rect[1]) # (x,y)
            face_rect_size = (rect[2] - rect[0], rect[3] - rect[1]) # (width, height)
            face_rect_center = (face_rect_origin[0] + face_rect_size[0] * 0.5,
                                face_rect_origin[1] + face_rect_size[1] * 0.5)

            rospy.loginfo("face detected at (x, y) = (%d, %d)" % face_rect_center)

            img_center = (img_size[0] * 0.5, img_size[1] * 0.5)
            face_relative_center = (face_rect_center[0] - img_center[0],
                                    face_rect_center[1] - img_center[1])

            pub_msg = Twist()
            pub_msg.linear.x  = face_relative_center[1] / img_center[1]
            pub_msg.angular.z = face_relative_center[0] / img_center[0]

            # this is the color of lines 
            color = (0, 255, 255)
            # draw a rectangle
            #cv2.rectangle(img_mat, (0,0), (100,200), color, 2)
            # draw a line
            #cv2.line(img_mat, (0,0), (300,300), color, 5)
            # draw a circle
            #cv2.circle(img_mat, (100,100), 100, color, 2)
  	    # and draw ellipses
	    cx = int(face_rect_center[0])
	    cy = int(face_rect_center[1])
            w = abs(int(face_rect_size[0]/2))
	    h = abs(int(w*1.2))
  	    cv2.ellipse(img_mat,(cx,cy),(w,h),0,0,360,color,2)
	    cv2.ellipse(img_mat,(cx,abs(int(cy+h/3))),(abs(int(w/2)),abs(int(h/4))), 0,0,180,color,-1)
	    cv2.circle(img_mat, (abs(int(cx-w/3)),abs(int(cy-h/4))), abs(int(w/8)), color, -1)
	    cv2.circle(img_mat, (abs(int(cx+w/3)),abs(int(cy-h/4))), abs(int(w/8)), color, -1)

            pub_debug_img_msg = self.bridge.cv2_to_imgmsg(img_mat, encoding="bgr8")
            self.debug_image_publisher.publish(pub_debug_img_msg)

            self.face_pose_publisher.publish(pub_msg)
        else:
            rospy.loginfo("no face detected.")

if __name__ == '__main__':

    rospy.init_node("test")
    face_detector = FaceDetectorMonoNode()
    rospy.spin()
