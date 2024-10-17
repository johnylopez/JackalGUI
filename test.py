import sys
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget, QPushButton
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Create a central widget
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        # Create a layout
        layout = QVBoxLayout(self.central_widget)

        # Load the background image
        self.background_label = QLabel(self)
        self.background_pixmap = QPixmap('/home/administrator/jackalgui/JackalGUI/assets/jackalGuiBackground.png')  # Replace with your image path
        self.background_label.setPixmap(self.background_pixmap)
        self.background_label.setScaledContents(True)
        self.background_label.setFixedSize(1200, 700)  # Set the size of the label

        # Add the QLabel to the layout
        layout.addWidget(self.background_label, alignment=Qt.AlignCenter)

        # Create a button
        self.button = QPushButton("Click me", self)
        # self.button.setGeometry(100, 100, 100, 30)  # Set position and size

        # Set the layout margins to zero to avoid spacing issues
        layout.setContentsMargins(0, 0, 0, 0)

        self.setWindowTitle("Background Image Example")
        self.setFixedSize(1200, 700)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())