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
	
	# 7. convert to grayscale image
        img_grey = cv2.cvtColor(img_mat, cv2.cv.CV_BGR2GRAY)
	
        # 7. extract edges with Canny's method
	img_canny=cv2.Canny(img_grey,50,150)
	
	# 8. convert to bgr again
        img_out = cv2.cvtColor(img_canny, cv2.cv.CV_GRAY2BGR)
	
        # 12. publish debug image
	color=(255,0,0)
        pub_debug_img_msg = self.bridge.cv2_to_imgmsg(img_out, encoding="bgr8")
        self.debug_image_publisher.publish(pub_debug_img_msg)

if __name__ == '__main__':
    # 3. At first, we must set node name, and register to the master.
    # In this case, node name is face_detector_mono
    rospy.init_node("face_detector_mono")
    face_detector = FaceDetectorMonoNode()

    # wait for message comming
    rospy.spin()
