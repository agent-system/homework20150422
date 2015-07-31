#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Ryo KOYAMA <koyama@jsk.imi.i.u-tokyo.ac.jp>

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

class MIRROR(object):
    """
This class has feature to subscribe "image" and detect people face,
then publishes those positions as geometry_msgs/Twist message.
"""

    def __init__(self):#constructor
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
        # 6. convert ROS sensor_msgs/Image to OpenCV format
        img_mat = self.bridge.imgmsg_to_cv2(msg)

        # 7. convert image to grey image
        img_flip = cv2.flip(img_mat, 1)

        # 12. publish debug image
        pub_debug_img_msg = self.bridge.cv2_to_imgmsg(img_flip, encoding="bgr8")
        self.debug_image_publisher.publish(pub_debug_img_msg)

if __name__ == '__main__':
    # 3. At first, we must set node name, and register to the master.
    # In this case, node name is face_detector_mono
    rospy.init_node("monokuro_camera")
    mirror = MIRROR()

    # wait for message comming
    rospy.spin()
