#!/usr/bin/env python

import rospy

from sensor_msgs.msg import *
from geometry_msgs.msg import *

from cv_bridge import CvBridge

import cv2

class edge_det(object):

    def __init__(self):
        try:
            self.bridge = CvBridge()

        except Exception as e:
            rospy.logerr("error: %s" % e)
        self.image_suscriber = rospy.Subscriber("image" , Image, self.mycallback)
        
        self.image_publisher = rospy.Publisher("edge_image", Image)



    def mycallback(self, msg):
        img_size = (msg.width, msg.height)


        img_mat = self.bridge.imgmsg_to_cv2(msg)
        

        img_grey = cv2.cvtColor(img_mat, cv2.cv.CV_BGR2GRAY)


        edges = cv2.Canny(img_grey,100,200)

        pub_debug_img_msg = self.bridge.cv2_to_imgmsg(edges , encoding="passthrough")

        self.image_publisher.publish(pub_debug_img_msg)

if __name__ == '__main__':
    rospy.init_node("edge_detect")
    edge_detect = edge_det()
    rospy.spin()


