import sys
from PySide6.QtCore import Qt
from PySide6.QtGui import QPalette, QColor
from PySide6.QtWidgets import (
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
    QFrame
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

        self.green_background = QWidget()
        self.green_background.setStyleSheet("background-color: green;")
        self.green_background.setGeometry(0,0,400,400)
        self.green_background.setFixedSize(600,400)

        self.mainLayout.addWidget(self.green_background)

        
        self.createInfoScreen()

        self.mainLayout.addWidget(self.page2)


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
        button1 = QPushButton("TF Model")
        button1.setFixedSize(100,50)
        buttonLayout.addWidget(button1)

        button2 = QPushButton("TensorRT Mode")
        button2.setFixedSize(100,50)
        buttonLayout.addWidget(button2)

        button3 = QPushButton("YoloV7 Model")
        button3.setFixedSize(100,50)
        buttonLayout.addWidget(button3)

        buttonWidget.setLayout(buttonLayout)
        layout.addWidget(buttonWidget)

        #TERMINAL
        terminalWidget = QWidget()
        terminalLayout = QVBoxLayout()
        infoLayout.setSpacing(0)
        terminalLabel = QLabel("Console Output")
        terminalLayout.addWidget(terminalLabel)
        terminal_output = QPlainTextEdit()
        terminal_output.setReadOnly(True)
        terminal_output.setFixedSize(500,200)
        terminal_output.setStyleSheet("background-color: black; color: white;")
        terminal_output.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        terminal_output.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        terminal_output.appendPlainText("When I die, I want this song playing in the background. I want to be sitting on a bench, staring at the sea, remembering the time she made me the happiest. Just replaying what I once had, what I'll only see in my mind, a treasure for no one but me.This man is from an era where you help others in need and don't expect anything in return or any recognition. This is how it should be. It's how it used to be once. We need to get back to those times. If you don't stand for something you'll fall for anything When I die, I want this song playing in the background. I want to be sitting on a bench, staring at the sea, remembering the time she made me the happiest. Just replaying what I once had, what I'll only see in my mind, a treasure for no one but me.This man is from an era where you help others in need and don't expect anything in return or any recognition. This is how it should be. It's how it used to be once. We need to get back to those times. If you don't stand for something you'll fall for anything When I die, I want this song playing in the background. I want to be sitting on a bench, staring at the sea, remembering the time she made me the happiest. Just replaying what I once had, what I'll only see in my mind, a treasure for no one but me.This man is from an era where you help others in need and don't expect anything in return or any recognition. This is how it should be. It's how it used to be once. We need to get back to those times. If you don't stand for something you'll fall for anything  ")
        terminalLayout.addWidget(terminal_output,alignment=Qt.AlignTop)
        terminalWidget.setLayout(terminalLayout)
        layout.addWidget(terminalWidget,alignment=Qt.AlignLeft)

        

        button = QPushButton("Bottom Aligned Button")
        layout.addWidget(button, alignment=Qt.AlignCenter)
        
        # Set the layout for the page
        self.page2.setLayout(layout)
        self.page2.setFixedSize(600,600)

app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()