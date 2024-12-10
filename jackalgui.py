import sys
import os
import subprocess
import rospy
import math
import cv2
from cv_bridge import CvBridge
from datetime import datetime
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from jackal_msgs.msg import Feedback
from imageHandler import ROSImageSubscriber
from report_generation import create_log, generateReport
from nav_msgs.msg import Odometry
from feedbackHandler import ROSJackalMesssagesSubscriber, ROSJackalDiagnosticMesssagesSubscriber, ROSJackalWifiConnectedSubscriber
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

        self.setWindowTitle("Clearpath Jackal Control Center")
        self.mainLayout = QHBoxLayout()

        self.backgroundLabel = QLabel(self)
        backgroundPixmap = QPixmap("/home/administrator/jackalgui/JackalGUI/assets/jackalGuiBackground2.png")  
        self.backgroundLabel.setPixmap(backgroundPixmap)
        self.backgroundLabel.setScaledContents(True)  
        self.backgroundLabel.setFixedSize(1200, 700)  

        widget = QWidget()
        widget.setLayout(self.mainLayout)
        self.setCentralWidget(widget)
        self.setFixedSize(1200,700)
        
        self.terminal_output = QPlainTextEdit()
        self.camera_process = None
        self.yolo_process = None
        self.culvertai_pytorch_process = None
        self.pipewatchai_tensorflow_process = None
        self.time = datetime.now()
        self.cameras_dict = {}

        self.createImageScreen()
        self.createInfoScreen()

        def feedback_center_callback(data):
            self.motor1_label.setText("Motor #1 Temp: " + str(math.ceil(data.drivers[0].motor_temperature)) + "C")
            self.bridge1_label.setText("Bridge #1 Temp: " + str(math.ceil(data.drivers[0].bridge_temperature)) + "C")
            if data.drivers[0].driver_fault == False:
                self.motor1_status_label.setText("Motor #1 Status: Active")
            else:
                self.motor1_status_label.setText("Motor #1 Status: Faulty")
            
            self.motor2_label.setText("Motor #2 Temp: " + str(math.ceil(data.drivers[1].motor_temperature)) + "C")
            self.bridge2_label.setText("Bridge #2 Temp: " + str(math.ceil(data.drivers[1].bridge_temperature)) + "C")
            if data.drivers[1].driver_fault == False:
                self.motor2_status_label.setText("Motor #2 Status: Active")
            else:
                self.motor2_status_label.setText("Motor #2 Status: Faulty")
            
            self.mcu_temp_label.setText("MCU Temp: " + str(math.ceil(data.mcu_temperature)) + "C")
            self.pcb_temp_label.setText("PCB Temp: " + str(math.ceil(data.pcb_temperature)) + "C")
        self.feedbackListener = ROSJackalMesssagesSubscriber(feedback_center_callback, '/feedback', 'feedback_node')

        def image_callback(pixmap, cv_image):
            self.update_image(pixmap)
            # self.calculate_and_show_fps()   
        self.cameraListener = ROSImageSubscriber(image_callback, '/axis/image_raw/compressed', CompressedImage)

        
        self.global_cameraListener = None

        def general_center_callback(data):
            if data.status[0].name == "jackal_node: General":
                self.label1.setText("Systems General: " + data.status[0].message)
                self.label2.setText("Battery: " + data.status[1].message)
                self.label3.setText("Battery Voltage: " + str(math.ceil(float(data.status[1].values[0].value))) + "V")
                self.label4.setText("Current Consumption: " + data.status[3].message)
                self.label5.setText("Power Consumption: " + data.status[4].message)
                # print(data.status[3].values)
                # print("\n")
        self.diagnosticListener = ROSJackalDiagnosticMesssagesSubscriber(general_center_callback, '/diagnostics')

        def wifi_callback(data):
            if data.data == True:
                self.wifi_label.setText("Wifi: Connected")
            else:
                self.wifi_label.setText("Wifi: Not Connected")
        self.wifiListener = ROSJackalWifiConnectedSubscriber(wifi_callback, '/wifi_connected')

        self.previous_x = 0.0
        self.previous_y = 0.0
        self.current_position = 0.0

        self.detections_subscriber = None
        self.odometry_subscriber = None

        self.current_image = None
        self.bridge = CvBridge()

        self.deficiencies_log = {}
        self.inspection_mode = False
    
    def closeEvent(self, event):
        rospy.signal_shutdown("Closing the application")
        event.accept()
    
    def global_image_callback(self,pixmap,cv_image):
        self.current_image = cv_image

    def odom_callback(self, msg):
        print(self.current_position)
        print(self.previous_x)
        print(self.previous_y)
        global previous_x, previous_y

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        delta_x = current_x - self.previous_x
        delta_y = current_y - self.previous_y
        distance_moved = math.sqrt(delta_x**2 + delta_y**2)

        distance_feet = distance_moved * 3.28084

        if delta_x > 0 or delta_y > 0:
            self.current_position += distance_feet
        else:
            self.current_position -= distance_feet

        self.position_label.setText(f"{self.current_position:.1f}ft")
        self.previous_x = current_x
        self.previous_y = current_y

    def detections_callback(self, msg):
        detections = msg.data.strip(", ").strip()
        if(detections != "" and self.inspection_mode):
            self.deficiencies_log[str(round(self.current_position, 1))] = {"detections": detections, "image": self.current_image}
        self.detections_label.setText("Deficiencies: " + detections)

    def createInfoScreen(self):
        self.page2 = QWidget()
        layout = QVBoxLayout()
        
        #INFO CENTER
        generalInfoWidget = QWidget()
        generalInfoLayout = QHBoxLayout()

        infoWidget = QWidget()
        infoLayout = QVBoxLayout()
        infoLayout.setSpacing(0)
        label = QLabel("Clearpath Jackal Control Center")
        infoLayout.addWidget(label)
        self.label1 = QLabel("Systems General: ")
        infoLayout.addWidget(self.label1)
        self.label2 = QLabel("Battery: ")
        infoLayout.addWidget(self.label2)
        self.label3 = QLabel("Battery Voltage: ")
        infoLayout.addWidget(self.label3)
        self.label4 = QLabel("Current Consumption: ")
        infoLayout.addWidget(self.label4)
        self.label5 = QLabel("Power Consumption: ")
        infoLayout.addWidget(self.label5)
        infoWidget.setLayout(infoLayout)
        
        tempInfoWidget = QWidget()
        tempInfoLayout = QVBoxLayout()
        tempInfoLayout.setSpacing(0)
        self.wifi_label = QLabel("Wifi: ")
        tempInfoLayout.addWidget(self.wifi_label)
        self.motor1_label = QLabel("Motor #1 Temp:")
        tempInfoLayout.addWidget(self.motor1_label)
        self.bridge1_label = QLabel("Bridge #1 Temp:")
        tempInfoLayout.addWidget(self.bridge1_label)
        self.motor1_status_label = QLabel("Motor #1 Status:")
        tempInfoLayout.addWidget(self.motor1_status_label)

        self.motor2_label = QLabel("Motor #2 Temp")
        tempInfoLayout.addWidget(self.motor2_label)
        self.bridge2_label = QLabel("Bridge #2 Temp:")
        tempInfoLayout.addWidget(self.bridge2_label)
        self.motor2_status_label = QLabel("Motor #2 Status:")
        tempInfoLayout.addWidget(self.motor2_status_label)
        self.mcu_temp_label = QLabel("MCU Temp:")
        tempInfoLayout.addWidget(self.mcu_temp_label)
        self.pcb_temp_label = QLabel("PCB Temp:")
        tempInfoLayout.addWidget(self.pcb_temp_label)
        tempInfoWidget.setLayout(tempInfoLayout)

        generalInfoLayout.addWidget(infoWidget)
        generalInfoLayout.addWidget(tempInfoWidget)
        generalInfoWidget.setLayout(generalInfoLayout)

        generalInfoWidget.setStyleSheet("border: 1px solid black; background-color: white;")

        layout.addWidget(generalInfoWidget, alignment= Qt.AlignCenter)

        #LOAD MODEL BUTTONS
        buttonWidget = QWidget()
        buttonLayout = QHBoxLayout()
        self.button1 = QPushButton("PipeWatchAI TF")
        self.button1.setFixedSize(120,50)
        self.button1.clicked.connect(self.launch_culvertai_tensorflow)
        buttonLayout.addWidget(self.button1)

        self.button2 = QPushButton("PipeWatchAI PT")
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
        layout.addWidget(buttonWidget, alignment=Qt.AlignCenter)

        #TERMINAL
        terminalWidget = QWidget()
        terminalLayout = QVBoxLayout()
        infoLayout.setSpacing(0)
        terminalLabel = QLabel("Messages")
        terminalLabel.setStyleSheet("Color: white;")
        terminalLayout.addWidget(terminalLabel, alignment=Qt.AlignRight)
        
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setFixedSize(550,300)
        self.terminal_output.setStyleSheet("background-color: black; color: white;")
        self.terminal_output.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.terminal_output.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        terminalLayout.addWidget(self.terminal_output,alignment=Qt.AlignCenter)
        terminalWidget.setLayout(terminalLayout)
        layout.addWidget(terminalWidget,alignment=Qt.AlignLeft)
        
        # Set the layout for the page
        self.page2.setLayout(layout)
        self.page2.setFixedSize(580,600)
        self.mainLayout.addWidget(self.page2, alignment=Qt.AlignRight)

    def createImageScreen(self):
        topics = rospy.get_published_topics()
        cameras = []
        for topic in topics:
            if topic[1] == 'sensor_msgs/Image' or topic[1] == 'sensor_msgs/CompressedImage':
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
        title.setStyleSheet("Color: white; ")
        title.setFixedSize(90,20)

        self.fps = QLabel(self)
        self.fps.setFixedSize(70,20)

        self.inspection_widget = QWidget()
        self.inspection_layout = QHBoxLayout()
        self.position_label = QLabel(self)
        self.position_label.setFixedSize(50,20)
        self.position_label.setStyleSheet("font-size: 24px;")
        self.detections_label = QLabel(self)
        self.detections_label.setFixedSize(500,20)
        self.detections_label.setStyleSheet("font-size: 24px;")
        self.inspection_button = QPushButton("Start Inspection")
        self.inspection_button.setFixedSize(150,30)
        self.inspection_button.clicked.connect(self.start_inspection)
        self.inspection_widget.setFixedSize(600,40)
        self.inspection_widget.setLayout(self.inspection_layout)
        self.inspection_layout.addWidget(self.position_label)
        self.inspection_layout.addWidget(self.detections_label)
        self.inspection_layout.addWidget(self.inspection_button)
        self.position_label.hide()
        self.detections_label.hide()
        self.inspection_button.hide()


        self.image = QLabel(self)
        self.image.setFixedSize(600,400)
        self.image.setStyleSheet("margin-top: 20px;")
        self.imageLayout.addWidget(title)
        self.imageLayout.addWidget(self.comboBackground)
        self.imageLayout.addWidget(self.image, alignment=Qt.AlignCenter)
        self.imageLayout.addWidget(self.inspection_widget)
        self.image_background.setFixedSize(600,530)
        self.image_background.setLayout(self.imageLayout)
        self.mainLayout.addWidget(self.image_background, alignment=Qt.AlignLeft)

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
            self.terminal_output.appendPlainText("Launching PipeWatchAI on pytorch...")
            self.position_label.show()
            self.detections_label.show()
            self.inspection_button.show()
            self.button2.setText("Close PipeWatch")
            self.button2.clicked.disconnect()
            self.button2.clicked.connect(self.close_culvert_pytorch_process)
        else:
            self.terminal_output.appendPlainText("Raw Image must be initialized before launching PipeWatchAI")
    
    def launch_culvertai_tensorflow(self):
        if self.camera_process and self.camera_process.poll() != 0:
            command = ["roslaunch", "culvertai_ros", "culvertai.launch"]
            self.pipewatchai_tensorflow_process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.terminal_output.appendPlainText("Launching PipeWatchAI on Tensorflow...")
            self.button1.setText("Close TF")
            self.button1.clicked.disconnect()
            self.button1.clicked.connect(self.close_culvert_tensorflow_process)
        else:
            self.terminal_output.appendPlainText("Raw Image must be initialized before launching PipeWatchAI on Tensorflow")

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

    def close_culvert_tensorflow_process(self):
        if self.pipewatchai_tensorflow_process:
            self.pipewatchai_tensorflow_process.terminate()
            self.button1.setText("PipeWatchAI TF")
            self.terminal_output.appendPlainText("PipeWatchAI Tensorflow closed")
            self.fps.setText("")
            self.button1.clicked.disconnect()
            self.button1.clicked.connect(self.launch_culvertai_tensorflow)

    def close_culvert_pytorch_process(self):
        if self.culvertai_pytorch_process:
            self.culvertai_pytorch_process.terminate()
            self.button2.setText("PipeWatchAI PT")
            self.terminal_output.appendPlainText("PipeWatchAI Pytorch closed")

            self.position_label.hide()
            self.detections_label.hide()
            self.inspection_button.hide()

            self.detections_label.setText("")
            if self.detections_subscriber:
                self.detections_subscriber.unregister()

            self.detections_subscriber = None
            self.fps.setText("")
            self.button2.clicked.disconnect()
            self.button2.clicked.connect(self.launch_culvertai_pytorch)
            self.stop_inspection()

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
        self.cameras_dict = {}
        for topic in topics:
            if topic[1] == 'sensor_msgs/Image' or topic[1] == 'sensor_msgs/CompressedImage':
                self.cameras_dict[topic[0]] = topic[1]
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
        if selected_camera:
            self.cameraListener.replace_topic(selected_camera, self.cameras_dict[selected_camera])
            if selected_camera == "/culvertai/culvert_ai/visualization":
                self.detections_subscriber = rospy.Subscriber('culvertai/culvert_ai/detections', String, self.detections_callback)

    def update_image(self, pixmap):
        scaled_pixmap = pixmap.scaled(self.image.size(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
        self.image.setPixmap(scaled_pixmap)

    def start_inspection(self):
        self.inspection_mode = True
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.current_position = 0.0
        self.odometry_subscriber = rospy.Subscriber('odometry/filtered', Odometry, self.odom_callback)
        self.inspection_button.setText("Stop Inspection")
        self.global_cameraListener = ROSImageSubscriber(self.global_image_callback, '/camera/image_raw', Image)
        self.inspection_button.clicked.disconnect()
        self.inspection_button.clicked.connect(self.stop_inspection)

    def stop_inspection(self):
        if self.odometry_subscriber:
            self.odometry_subscriber.unregister()
            self.odometry_subscriber = None
            self.global_cameraListener.unsubscribe()
            self.global_cameraListener = None
            self.inspection_button.setText("Start Inspection")
            self.inspection_button.clicked.disconnect()
            self.inspection_button.clicked.connect(self.start_inspection)
            self.inspection_mode = False
            output = create_log(self.deficiencies_log)
            generateReport(output)
    

def main():
    rospy.init_node('jackalgui_node', anonymous=True)
    app = QApplication(sys.argv)
    window = MainWindow()

    window.show()
    sys.exit(app.exec())
    

if __name__ == '__main__':
    main() 