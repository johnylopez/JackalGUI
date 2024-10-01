import sys
import os
import subprocess
import rospy
from datetime import datetime
from sensor_msgs.msg import Image, CompressedImage
from jackal_msgs.msg import Feedback
from imageHandler import ROSImageSubscriber
from feedbackHandler import ROSJackalMesssagesSubscriber
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette, QColor, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QLineEdit,
    QMainWindow,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
    QPushButton,
    QSizePolicy,
    QStackedLayout,
    QTextEdit,
    QPlainTextEdit,
    QFrame,
    QComboBox
)

class Color(QWidget):

    def __init__(self, color):
        super(Color, self).__init__()
        self.setAutoFillBackground(True)

        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(color))
        self.setPalette(palette)

class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()

        self.setWindowTitle("My App")
        self.setStyleSheet("background-color: lightblue;")
        self.mainLayout = QHBoxLayout()

        def image_callback(pixmap):
            self.update_image(pixmap)
            self.calculate_and_show_fps()   
        self.cameraListener = ROSImageSubscriber(image_callback, '/camera/image_raw')

        def control_center_callback(data):
            print(data.pcb_temperature)
        self.feedbackListener = ROSJackalMesssagesSubscriber(control_center_callback, '/feedback', 'feedback_node')





        self.terminal_output = QPlainTextEdit()
        self.camera_process = None
        self.yolo_process = None
        self.culvertai_pytorch_process = None
        self.time = datetime.now()

        self.createImageScreen()
        self.createInfoScreen()

        widget = QWidget()
        widget.setLayout(self.mainLayout)
        self.setCentralWidget(widget)
        self.setFixedSize(1200,700)

    def createInfoScreen(self):
        self.page2 = QWidget()
        layout = QVBoxLayout()
        
        # self.page2.setStyleSheet("border: 2px solid black;")  
        
        #INFO CENTER
        infoWidget = QWidget()
        infoLayout = QVBoxLayout()
        infoLayout.setSpacing(0)
        label = QLabel("Clearpath Jackal Control Center")
        infoLayout.addWidget(label)
        label1 = QLabel("Velocity: ")
        infoLayout.addWidget(label1)
        label2 = QLabel("X Pos:")
        infoLayout.addWidget(label2)
        label3 = QLabel("Y Pos:")
        infoLayout.addWidget(label3)
        label4 = QLabel("Etc: ")
        infoLayout.addWidget(label4)
        infoWidget.setLayout(infoLayout)
        layout.addWidget(infoWidget, alignment= Qt.AlignLeft)

        #LOAD MODEL BUTTONS
        buttonWidget = QWidget()
        buttonLayout = QHBoxLayout()
        self.button1 = QPushButton("TF Model")
        self.button1.setFixedSize(120,50)
        buttonLayout.addWidget(self.button1)

        self.button2 = QPushButton("CulvertAI PT")
        self.button2.setFixedSize(120,50)
        self.button2.clicked.connect(self.launch_culvertai_pytorch)
        buttonLayout.addWidget(self.button2)

        self.button3 = QPushButton("YoloV7 Model")
        self.button3.setFixedSize(120,50)
        self.button3.clicked.connect(self.launch_yolo)
        buttonLayout.addWidget(self.button3)

        self.button4 = QPushButton("Raw Image")
        self.button4.setFixedSize(120,50)
        self.button4.clicked.connect(self.launch_raw_camera)
        buttonLayout.addWidget(self.button4)

        buttonWidget.setLayout(buttonLayout)
        layout.addWidget(buttonWidget)

        #TERMINAL
        terminalWidget = QWidget()
        terminalLayout = QVBoxLayout()
        infoLayout.setSpacing(0)
        terminalLabel = QLabel("Console Output")
        terminalLayout.addWidget(terminalLabel)
        
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setFixedSize(550,200)
        self.terminal_output.setStyleSheet("background-color: black; color: white;")
        self.terminal_output.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.terminal_output.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        terminalLayout.addWidget(self.terminal_output,alignment=Qt.AlignTop)
        terminalWidget.setLayout(terminalLayout)
        layout.addWidget(terminalWidget,alignment=Qt.AlignLeft)

        button = QPushButton("Bottom Aligned Button")
        layout.addWidget(button, alignment=Qt.AlignCenter)
        
        # Set the layout for the page
        self.page2.setLayout(layout)
        self.page2.setFixedSize(600,600)
        self.mainLayout.addWidget(self.page2)


    def createImageScreen(self):
        topics = rospy.get_published_topics()
        cameras = []
        for topic in topics:
            if topic[1] == 'sensor_msgs/Image':
                cameras.append(topic[0])
            
        #IMAGE VIEWER
        self.image_background = QWidget()
        self.imageLayout = QVBoxLayout()
        self.comboBackground = QWidget()
        self.comboLayout = QHBoxLayout()

        self.combo_box = QComboBox(self)
        self.combo_box.setFixedSize(400,30)
        self.combo_box.addItems(cameras)
        self.combo_box.currentIndexChanged.connect(self.update_camera)
        self.comboLayout.addWidget(self.combo_box)

        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.setFixedSize(70,30)
        self.refresh_button.clicked.connect(self.update_items)
        self.comboLayout.addWidget(self.refresh_button)
        self.comboBackground.setFixedSize(600,40)
        self.comboBackground.setLayout(self.comboLayout)

        title = QLabel("CAMERAS: ")
        title.setFixedSize(90,20)

        self.fps = QLabel(self)
        self.fps.setFixedSize(70,20)

        self.image = QLabel(self)
        self.imageLayout.addWidget(title)
        self.imageLayout.addWidget(self.comboBackground)
        self.imageLayout.addWidget(self.image)
        self.imageLayout.addWidget(self.fps)
        self.image_background.setGeometry(0,0,400,400)
        self.image_background.setFixedSize(600,500)
        self.image_background.setLayout(self.imageLayout)

        self.mainLayout.addWidget(self.image_background)

    def launch_raw_camera(self):
        command = ["roslaunch", "compressed_to_raw", "compressed_to_raw.launch"]
        self.camera_process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.terminal_output.appendPlainText("Raw Image launched succesully")
        self.button4.setText("Close Raw Image")
        self.button4.clicked.disconnect()
        self.button4.clicked.connect(self.close_camera_process)

    def launch_culvertai_pytorch(self):
        if self.camera_process and self.camera_process.poll() != 0:
            command = ["roslaunch", "culvertai_pytorch", "culvertaipytorch.launch"]
            self.culvertai_pytorch_process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.terminal_output.appendPlainText("Launching Culvert and Sewer Inspection on pytorch...")
            self.button2.setText("Close Culvert and Sewer inspection on Pytorch")
            self.button2.clicked.disconnect()
            self.button2.clicked.connect(self.close_culvert_pytorch_process)
        else:
            self.terminal_output.appendPlainText("Raw Image must be initialized before launching CulvertAI")
    
    def launch_yolo(self):
        if self.camera_process and self.camera_process.poll() != 0:
            command = ["roslaunch", "yolov7_ros", "yolov7.launch"]
            self.yolo_process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.terminal_output.appendPlainText("Launching YoloV7...")
            self.button3.setText("Close YoloV7")
            self.button3.clicked.disconnect()
            self.button3.clicked.connect(self.close_yolo_process)
        else:
            self.terminal_output.appendPlainText("Raw Image must be initialized before launching YoloV7")

    def close_culvert_pytorch_process(self):
        if self.culvertai_pytorch_process:
            self.culvertai_pytorch_process.terminate()
            self.button2.setText("CulvertAI PT")
            self.terminal_output.appendPlainText("CulvertAI Pytorch closed")
            self.fps.setText("")
            self.button2.clicked.disconnect()
            self.button2.clicked.connect(self.launch_culvertai_pytorch)

    def close_camera_process(self):
        if self.camera_process:
            self.camera_process.terminate()
            self.button4.setText("Raw Image")
            self.terminal_output.appendPlainText("Raw Image closed")
            self.fps.setText("")
            self.button4.clicked.disconnect()
            self.button4.clicked.connect(self.launch_raw_camera)

    def close_yolo_process(self):
        if self.yolo_process:
            self.yolo_process.terminate()
            self.button3.setText("Yolo V7")
            self.terminal_output.appendPlainText("YoloV7 closed")
            self.fps.setText("")
            self.button3.clicked.disconnect()
            self.button3.clicked.connect(self.launch_yolo)

    def update_items(self):
        topics = rospy.get_published_topics()
        cameras = []
        for topic in topics:
            if topic[1] == 'sensor_msgs/Image':
                cameras.append(topic[0])
        self.combo_box.clear()
        self.combo_box.addItems(cameras)

    def calculate_and_show_fps(self): 
        # FPS
        timediff = datetime.now() - self.time
        self.time = datetime.now()
        fps = round(1.0 / timediff.total_seconds(),2)
        self.fps.setText("FPS: " + str(fps))

    def update_camera(self):
        selected_camera = self.combo_box.currentText()
        self.cameraListener.replace_topic(selected_camera)

    def update_image(self, pixmap):
        scaled_pixmap = pixmap.scaled(self.image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image.setPixmap(scaled_pixmap)
    

def main():
    app = QApplication(sys.argv)
    window = MainWindow()

    window.show()
    sys.exit(app.exec())
    

if __name__ == '__main__':
    main() 