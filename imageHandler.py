import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from PyQt5.QtGui import QImage
import os

class ROSImageSubscriber:
    def __init__(self):
        # os.environ['ROS_MASTER_URI'] = 'http://uono01-1:11311'
        rospy.init_node('image_subscriber_node')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.image = None

    def callback(self, msg):
        # Convert ROS Image message to OpenCV format
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def get_image(self):
        return self.image



def cv_image_to_qimage(cv_image):
    height, width, channels = cv_image.shape
    bytes_per_line = 3 * width
    q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
    return q_image.rgbSwapped()