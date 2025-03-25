import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QPushButton, QFileDialog
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt, QPoint

class VideoPlayer(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        
        self.cap = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.nextFrameSlot)
        self.mouse_positions = []

    def initUI(self):
        self.layout = QVBoxLayout()
        
        self.video_label = QLabel()
        self.layout.addWidget(self.video_label)
        
        self.btn_open = QPushButton("Open Video")
        self.btn_open.clicked.connect(self.openVideo)
        self.layout.addWidget(self.btn_open)
        
        self.setLayout(self.layout)
        
        self.setWindowTitle("Video Player with Mouse Tracking")
        self.show()
        
    def openVideo(self):
        video_path, _ = QFileDialog.getOpenFileName(self, "Open Video")
        if video_path:
            self.cap = cv2.VideoCapture(video_path)
            self.timer.start(30)  # 30 ms interval for ~30 fps

    def nextFrameSlot(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qimg))
        else:
            self.timer.stop()

    def mouseMoveEvent(self, event):
        pos = event.pos()
        self.mouse_positions.append((pos.x(), pos.y()))
        print(f"Mouse position in video: {pos.x()}, {pos.y()}")

    def closeEvent(self, event):
        if self.cap:
            self.cap.release()
        print("Mouse positions recorded during video playback:")
        print(self.mouse_positions)
        super().closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    player = VideoPlayer()
    sys.exit(app.exec_())
