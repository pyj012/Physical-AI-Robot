import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from GUI.mainPage import *
from GUI.cameraPage import *

class ApplicationPages():
    def __init__(self,stack):
        self.stack = stack 
        self.mainpage = MainPage(self.stack)
        self.camerapage=CameraPage(self.stack)