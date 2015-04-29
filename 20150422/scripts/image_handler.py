#!/usr/bin/env python
# -*- coding: utf-8 -*-
# By LI XU (48-156623)
# Use 'cv2.medianBlur' to filter income msg, then publish it.


import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

import cv2

cascade_path = "/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_alt2.xml"

class ImageHandlerNode(object):
    def __init__(self):
        try:
            self.bridge = CvBridge()
            self.cascade = cv2.CascadeClassifier(cascade_path)
        except Exception as e:
            rospy.logerr("Error: %s" % e)

        self.image_subscriber = rospy.Subscriber("image", Image, self.image_callback)
        self.debug_image_publisher = rospy.Publisher("debug_image", Image)

    def image_callback(self, msg):
        img_size = (msg.width, msg.height)

        img_mat = self.bridge.imgmsg_to_cv2(msg)
        # Use medianBlur filter
        img_new = cv2.medianBlur(img_mat, 19)
        
        pub_debug_img_msg = self.bridge.cv2_to_imgmsg(img_new, encoding="bgr8")
        self.debug_image_publisher.publish(pub_debug_img_msg)

if __name__ == '__main__':
    rospy.init_node("image_handler")
    image_handler_node = ImageHandlerNode()
    rospy.spin()

