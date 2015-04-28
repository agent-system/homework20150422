#!/usr/bin/env python                                                          
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import Image # This imports sensor_msgs/Image
from geometry_msgs.msg import Twist # This imports geometry_msgs/Twist

from cv_bridge import CvBridge

import cv2 # this imports opencv python interface

class edge_detect(object):

    def __init__(self):
    
        try:
            self.bridge = CvBridge()
        except Exception as e:
            rospy.logerr("Error: %s" % e)

        self.image_subscriber = rospy.Subscriber("image", Image, self.image_callback)

        self.face_pose_publisher = rospy.Publisher("twist", Twist, queue_size=1)
        self.debug_image_publisher = rospy.Publisher("debug_image", Image, queue_size = 1)

    def image_callback(self, msg):

        img_mat = self.bridge.imgmsg_to_cv2(msg)
        img_gray = cv2.cvtColor(img_mat, cv2.COLOR_BGR2GRAY)
        
        img_edge = cv2.Canny(img_gray, 50, 200)

        pub_msg = Twist()

        pub_debug_img_msg = self.bridge.cv2_to_imgmsg(img_edge, encoding="mono8")
        self.debug_image_publisher.publish(pub_debug_img_msg)
        self.face_pose_publisher.publish(pub_msg)

if __name__ == '__main__':

    rospy.init_node("edge_detect")
    mono = edge_detect()

    # wait for message comming
    rospy.spin()
