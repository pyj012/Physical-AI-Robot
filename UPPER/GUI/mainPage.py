import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from GUI.imgs.imgpath import *
import cv2
import numpy as np
from PIL.ImageQt import ImageQt
from PIL import Image

class MainPage():
    def __init__(self, stack):
        self.stack=stack
        self.MainPage = QWidget()                         # page_1 생성
        self.current_gif="IDLE"
        self.frame_cnt =0
        self.label = QLabel(self.MainPage)                  # page_2를 부모로 label_2 생성
        self.label.setAlignment(Qt.AlignCenter)           # 가운데 정렬
        self.label.setGeometry(0,0,1080,1080)                # 위치 및 크기 지정
        self.label.setScaledContents(True)

        # 동적 이미지 추가
        self.movie = QMovie(SLEEP_GIF, QByteArray())
        self.movie.setCacheMode(QMovie.CacheAll)
        self.label.setMovie(self.movie)
        self.movie.start()

        self.stack.addWidget(self.MainPage)                   # stack에 page_1 추가

    def frame_change(self, frame_number):
        if frame_number == self.movie.frameCount()-1:
            self.setIdleEMOJI()

    def setSleepEMOJI(self):
        self.current_gif = "SLEEP"
        self.movie.stop()
        self.movie = QMovie(SLEEP_GIF, QByteArray())
        self.movie.setCacheMode(QMovie.CacheAll)
        self.label.setMovie(self.movie)
        self.movie.start()

    def setIdleEMOJI(self):
        self.current_gif = "IDLE"
        self.movie.stop()
        self.movie = QMovie(IDLE_GIF, QByteArray())
        self.movie.setCacheMode(QMovie.CacheAll)
        self.label.setMovie(self.movie)
        self.movie.start()

    def setSmileEMOJI(self):
        self.current_gif = "SMILE"
        self.movie.stop()
        self.movie = QMovie(SMILE_GIF, QByteArray())
        self.movie.setCacheMode(QMovie.CacheAll)
        self.label.setMovie(self.movie)
        self.movie.start()

    def setAngryEMOJI(self):
        self.current_gif = "ANGRY"
        self.movie.stop()
        self.movie = QMovie(ANGRY_GIF, QByteArray())
        self.movie.setCacheMode(QMovie.CacheAll)
        self.label.setMovie(self.movie)
        self.movie.frameChanged.connect(self.frame_change)
        self.movie.start()

    def setVeryAngryEMOJI(self):
        self.current_gif = "VERY_ANGRY"
        self.movie.stop()
        self.movie = QMovie(VERY_ANGRY_GIF, QByteArray())
        self.movie.setCacheMode(QMovie.CacheAll)
        self.label.setMovie(self.movie)
        self.movie.frameChanged.connect(self.frame_change)
        self.movie.start()

    def setSearchEMOJI(self):
        self.current_gif = "SEARCH"
        self.movie.stop()
        self.movie = QMovie(SEARCH_GIF, QByteArray())
        self.movie.setCacheMode(QMovie.CacheAll)
        self.label.setMovie(self.movie)
        self.movie.frameChanged.connect(self.frame_change)
        self.movie.start()

    def setFunnyEMOJI(self):
        self.current_gif = "FUNNY"
        self.movie.stop()
        self.movie = QMovie(FUNNY_GIF, QByteArray())
        self.movie.setCacheMode(QMovie.CacheAll)
        self.label.setMovie(self.movie)
        self.movie.frameChanged.connect(self.frame_change)
        self.movie.start()

    def setSmilingEMOJI(self):
        self.current_gif = "SMILING"
        self.movie.stop()
        self.movie = QMovie(SMILING_GIF, QByteArray())
        self.movie.setCacheMode(QMovie.CacheAll)
        self.label.setMovie(self.movie)
        self.movie.frameChanged.connect(self.frame_change)
        self.movie.start()

    def setThinkingEMOJI(self):
        self.current_gif = "THINKING"
        self.movie.stop()
        self.movie = QMovie(THINKING_GIF, QByteArray())
        self.movie.setCacheMode(QMovie.CacheAll)
        self.label.setMovie(self.movie)
        self.movie.start()

    def stopEMOJI(self):
        self.movie.stop()
    def setImgLabel(self, img):
        try:
            # yourQImage=qimage2ndarray.array2qimage(img)
            # # np.frombuffer(buffer=pix.sample, dtype=mp.)
            # color_img= cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # height, width, channel = color_img.shape
            # bytesPerLine = 3 * width
            # qImg = QImage(color_img.data, width, height, bytesPerLine, QImage.Format_RGB888)
            pixmap = self.convertCvImage2QtImage(img)
            # self.pixmap.load(color_img)
            self.label.setPixmap(pixmap)
        except Exception as e:
            pass

    def convertCvImage2QtImage(self, img):
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        PIL_img = Image.fromarray(rgb_img).convert('RGB')
        return QPixmap.fromImage(ImageQt(PIL_img))
    
