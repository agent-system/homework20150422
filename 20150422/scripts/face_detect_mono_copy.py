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

    # 5. define callback function for image topic
    def image_callback(self, msg):
        img_size = (msg.width, msg.height)

        # 6. convert ROS sensor_msgs/Image to OpenCV format
        img_mat = self.bridge.imgmsg_to_cv2(msg)

        # 7. convert image to grey image
        img_grey = cv2.cvtColor(img_mat, cv2.cv.CV_BGR2GRAY)

        # 7. detect face in an image
        face_rects = self.cascade.detectMultiScale(img_grey,
                                                  scaleFactor=1.1,
                                                  minNeighbors=1,
                                                  minSize=(1,1))

        if len(face_rects) > 0:
            rect = face_rects[0] # use first rect
            face_rect_origin = (rect[0], rect[1]) # (x,y)
            face_rect_size = (rect[2] - rect[0], rect[3] - rect[1]) # (width, height)
            face_rect_center = (face_rect_origin[0] + face_rect_size[0] * 0.5,
                                face_rect_origin[1] + face_rect_size[1] * 0.5)

            # 8. logging face position
            rospy.loginfo("face detected at (x, y) = (%d, %d)" % face_rect_center)

            # 9. compute center of face relative to center of camera image
            #    Note that face_relative_center has value (-img_size/2 ~ +img_size/2)
            img_center = (img_size[0] * 0.5, img_size[1] * 0.5)
            face_relative_center = (face_rect_center[0] - img_center[0],
                                    face_rect_center[1] - img_center[1])

            # 10. make Twist message and set values
            pub_msg = Twist()
            pub_msg.linear.x  = face_relative_center[1] / img_center[1]
            pub_msg.angular.z = face_relative_center[0] / img_center[0]

            # 12. publish debug image
            color = (255, 0, 0)
            cv2.rectangle(img_mat, face_rect_origin,
                          (face_rect_origin[0] + face_rect_size[0], face_rect_origin[1] + face_rect_size[1]),
                          color, thickness=2)
            pub_debug_img_msg = self.bridge.cv2_to_imgmsg(img_mat, encoding="bgr8")
            self.debug_image_publisher.publish(pub_debug_img_msg)

            # 11. publish message
            self.face_pose_publisher.publish(pub_msg)
        else:
            rospy.loginfo("no face detected.")

if __name__ == '__main__':
    # 3. At first, we must set node name, and register to the master.
    # In this case, node name is face_detector_mono
    rospy.init_node("face_detector_mono")
    face_detector = FaceDetectorMonoNode()

    # wait for message comming
    rospy.spin()
