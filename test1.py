import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt5.QtCore import Qt
import sys

class ImageViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS Image Viewer')
        self.label = QLabel(self)
        self.setCentralWidget(self.label)
        self.bridge = CvBridge()
        # pixmap = QPixmap('/home/administrator/Downloads/test_image.png')
        # print(pixmap.shape)
        # self.label.setPixmap(pixmap)
        self.label.setGeometry(100, 100, 500, 800)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        qt_image = self.convert_cv_to_qt(cv_image)
        pixmap = QPixmap(qt_image)
        self.label.setPixmap(pixmap)
        self.label.adjustSize() 

    def convert_cv_to_qt(self, cv_image):
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_image.shape
        bytes_per_line = channels * width
        qt_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        return qt_image

def main():
    rospy.init_node('image_viewer_node', anonymous=True)
    app = QApplication([])
    viewer = ImageViewer()
    viewer.show()
    sys.exit(app.exec())
    
    # rospy.spin()

if __name__ == '__main__':
    main()