import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from PyQt5.QtGui import QImage, QPixmap
import os

class ROSImageSubscriber:
    def __init__(self, image_callback, image_topic):
        self.image_callback = image_callback
        self.image_topic = image_topic
        rospy.init_node('image_subscriber_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.callback)
        self.pixmap = None

    def replace_topic(self, topic):
        self.image_sub.unregister()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        qt_image = self.convert_cv_to_qt(cv_image)
        pixmap = QPixmap(qt_image)
        self.image_callback(pixmap)

    def convert_cv_to_qt(self, cv_image):
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_image.shape
        bytes_per_line = channels * width
        qt_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        return qt_image



