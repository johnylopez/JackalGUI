import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from PyQt5.QtGui import QImage, QPixmap
import os

class ROSImageSubscriber:
    def __init__(self, image_callback, image_topic, msg_type):
        self.image_callback = image_callback
        self.image_topic = image_topic
        self.bridge = CvBridge()
        self.msg_type = msg_type
        self.image_sub = rospy.Subscriber(self.image_topic, self.msg_type, self.callback)
        self.pixmap = None

    def replace_topic(self, topic, msg_type):
        if msg_type == 'sensor_msgs/CompressedImage': 
            self.msg_type = CompressedImage 
        else:
            self.msg_type = Image
            
        self.image_sub.unregister()
        self.image_sub = rospy.Subscriber(topic, self.msg_type, self.callback)

    def callback(self, msg):
        if isinstance(msg, Image):
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        elif isinstance(msg, CompressedImage):
            np_array = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        
        qt_image = self.convert_cv_to_qt(cv_image)
        pixmap = QPixmap(qt_image)
        self.image_callback(pixmap, cv_image)

    def convert_cv_to_qt(self, cv_image):
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_image.shape
        bytes_per_line = channels * width
        qt_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        return qt_image

    def unsubscribe(self):
        self.image_sub.unregister()


