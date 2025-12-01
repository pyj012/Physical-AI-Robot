import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from GUI.imgs.imgpath import *
import cv2
import numpy as np
from PIL.ImageQt import ImageQt
from PIL import Image
class CameraPage:
    def __init__(self, stack):
        self.stack = stack
        self.CameraPage = QWidget()
        self.label = QLabel(self.CameraPage)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setGeometry(0, 0, 1080, 1080)
        self.label.setScaledContents(True)
        self.stack.addWidget(self.CameraPage)
        
        # 미리 로드하는 코드는 그대로 두셔도 좋지만, 문제 해결의 핵심은 아닙니다.
        placeholder_pixmap = QPixmap(1080, 1080)
        placeholder_pixmap.fill(Qt.black)
        self.label.setPixmap(placeholder_pixmap)

    def setImgLabel(self, img):
        try:
            # 이제 이 함수는 '진짜 그림'을 반환하므로 안전합니다.
            pixmap = self.convert_cv_to_qt_safe(img)
            self.label.setPixmap(pixmap)
        except Exception as e:
            print(f"Error in setImgLabel: {e}")
                
    def convert_cv_to_qt_safe(self, cv_img):

        try:
            # BGR -> RGB 색상 변환
            rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w

            # 'memoryview' 오류를 해결하기 위해, 배열 자체를 복사한 후 데이터에 접근합니다.
            copied_image = rgb_image.copy()
            q_image = QImage(copied_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            return QPixmap.fromImage(q_image)
        except Exception as e:
            print(f"Error converting OpenCV image to QPixmap: {e}")
            return QPixmap() # 오류 발생 시 빈 Pixmap 반환


