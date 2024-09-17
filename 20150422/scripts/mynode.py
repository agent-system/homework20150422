
import rospy

from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist 

from cv_bridge import CvBridge

import cv2 

class mydetect_node(object):
    

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
	img_edge = cv2.Canny(img_grey,100,100)

	pub_debug_img_msg = self.bridge.cv2_to_imgmsg(img_edge, encoding="bgr8")
	self.debug_image_publisher.publish(pub_debug_img_msg)

if __name__ == '__main__':
    rospy.init_node("mydetectnode")
    mydetectnode = mydetect_node()

    rospy.spin()
