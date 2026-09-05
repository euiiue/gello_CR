from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QAction, QMessageBox, QMainWindow
from PyQt5 import QtWidgets
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot
from PyQt5.Qt import QLineEdit
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot
from PyQt5.Qt import QLineEdit
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import threading
import time
import math
import os
import sys
import numpy as np
import serial
import argparse
import threading
import json
import queue
import copy
import logging
from logging.handlers import RotatingFileHandler
import argparse
import threading
import socket
import json

import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QHBoxLayout
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
import numpy as np
import time
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent

# V2 package compatibility during staged migration of the legacy Qt entrypoint.
_V2_SRC_DIR = BASE_DIR / "src"
if str(_V2_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_V2_SRC_DIR))
from gello_cr.devices.cr3a import Cr3aConfig, Cr3aDevice
from gello_cr.devices.realsense import RealSenseRgbConfig, RealSenseRgbDevice
from gello_cr.recording.image_processing import crop_normalized_roi



# INEXBOT class
# 如果要引用子文件夹的一个内容还不行，就直接系统路径中包含这个文件夹
# from TEST_INEXBOT.nrc_interface import nrc_interface as aa
sys.path.append(str(BASE_DIR / 'TESTRobot_INEXBOT'))
import nrc_interface as aa
import sys
from PyQt5.QtWidgets import QApplication
#from Widget import WidgetApp  # 从 test1.py 导入 WidgetApp 类
import time
from PyQt5.QtCore import pyqtSignal , QThread
import threading
import ctypes
Size = 0
pos = aa.VectorDouble()
vmax = aa.VectorDouble()
amax = aa.VectorDouble()
jmax = aa.VectorDouble()
global ROBOT1_JointPOS # J0-J5 d will add gello_1 percent as gripper state
ROBOT1_JointPOS = [0,0,0,0,0,0,0]
global ROBOT1_TCPPOS # X,Y,Z,A,B,C and will add gello_1 percent as gripper state
ROBOT1_TCPPOS = [0,0,0,0,0,0,0]

# GELLO hand class
# 引用子文件夹的py文件，文件夹.文件名 即可（文件夹内部放一个空的__init__.py）
# import driver
# from TESTMaster_GELLO import driver
#from driver import DynamixelDriver
# RoArm-M2-Pro 使用原厂 ESP32 JSON/UART，不再使用 DynamixelDriver。

global GELLO_1
global GELLO_1_ids
#GELLO_1_ids=[3]
#GELLO_1_ids = [1,2,3,4,5,6] # control all motors
GELLO_1_ids = [1,2,3,4,5,6,7] # control all motors

global GELLO_1_Torque
#GELLO_1_Torque = [0,0,0,0,0,0]
GELLO_1_Torque = [0,0,0,0,0,0,0]
global GELLO_1_Position
GELLO_1_Position = [0,0,0,0,0,0,0]

global GELLO_1_Direction
#GELLO_1_Direction = [1.0,1.0,1.0,1.0,1.0,1.0,1.0]
GELLO_1_Direction = [1.0,1.0,-1.0,1.0,1.0,1.0,1.0]


# LINKERBOT Hand class
# 如果要引用子文件夹的一个内容还不行，就直接系统路径中包含这个文件夹
sys.path.append(str(BASE_DIR / 'TESTHand_LINKERBOT'))
from linker_hand_python_sdk.LinkerHand.linker_hand_api import LinkerHandApi
from linker_hand_python_sdk.LinkerHand.utils.color_msg import ColorMsg
from linker_hand_python_sdk.LinkerHand.utils.load_write_yaml import LoadWriteYaml

global Hand_1_Pos
Hand_1_Pos = [0,0,0,0,0,0,0]

global Hand_1_Vel
Hand_1_Vel = [0,0,0,0,0,0,0]

global Hand_1_Torque
Hand_1_Torque = [0,0,0,0,0,0,0]

global Hand_1_Error
Hand_1_Error = [0,0,0,0,0,0,0]


# WEIXUE Master Hand Class
sys.path.append(str(BASE_DIR / 'TESTRobot_WEIXUE'))
# MasterHand_WEIXUE.py 在被 import 时会立即打开 /dev/ttyUSB0 并进入死循环，
# 会抢占 RoArm 串口。旧 WEIXUE 页面保留，但不再自动导入该模块。
MasterHand_WEIXUE = None

global WEIXUE_1_CurrentJointPos
WEIXUE_1_CurrentJointPos = [0,0,0,0,0,0]

global WEIXUE_1_CurrentTCPPos
WEIXUE_1_CurrentTCPPos = [0,0,0,0,0,0]


# Force Sensor Class
sys.path.append(str(BASE_DIR / 'TESTSensor_ROBOTIQ'))
import Sensor_Robotiq

from teleop_runtime import (
    GelloController,
    Inverse3Controller,
    NrcRobotAdapter,
    O6Controller,
    O6_MOTOR_NAMES,
    RoArmSerialController,
    TeleopConfigStore,
    TeleopEngine,
)
from lerobot_recorder import LeRobotEpisodeRecorder


SHORTCUT_LABELS = {
    "prepare": "准备设备与双相机（不启动跟随）",
    "power_on": "CR5 单独上使能（不启动跟随）",
    "start_follow": "单独开始主从跟随（不自动上使能）",
    "stop_follow": "停止当前动作并保持从臂",
    "episode_start": "开始 Episode",
    "episode_save_next": "保存当前 Episode（不自动开始下一条）",
    "episode_discard_retry": "丢弃当前 Episode（不自动重录）",
    "home": "主从臂回 HOME",
    "emergency_stop": "软件紧急停止",
    "master_free": "从臂保持 / 主臂自由",
    "o6_open": "O6 张开手",
    "o6_grasp": "O6 抓取",
    "o6_light_grasp": "O6 轻微抓取",
    "o6_m5": "恢复 M5 控制 O6",
    "preset_a": "回到示教点 A",
    "preset_b": "回到示教点 B",
    "preset_c": "回到示教点 C",
    "preset_d": "回到示教点 D",
    "preset_save_a": "保存示教点 A",
    "preset_save_b": "保存示教点 B",
    "preset_save_c": "保存示教点 C",
    "preset_save_d": "保存示教点 D",
}

COMMON_SHORTCUT_IDS = (
    "prepare",
    "power_on",
    "start_follow",
    "stop_follow",
    "episode_start",
    "episode_save_next",
    "episode_discard_retry",
    "home",
    "emergency_stop",
)

PRESET_SHORTCUT_IDS = (
    "preset_a",
    "preset_b",
    "preset_c",
    "preset_d",
    "preset_save_a",
    "preset_save_b",
    "preset_save_c",
    "preset_save_d",
)


global ForceSensorData
ForceSensorData= [0,0,0,0,0,0] #fx,fy,fz,tx,ty,tz

# HDF5 Collect
global image_list_diagonal_view
image_list_diagonal_view = []

global image_list_diagonal_view_deepth
image_list_diagonal_view_deepth = []


global qpos_list
qpos_list = []

global action_list
action_list=[]

global qpos
qpos = [0,0,0,0,0,0,0,0] #j0-j7 + grippger if 6 axis leave 0 #从动机械臂的关节位置

global action
action = [0,0,0,0,0,0,0,0] #j0-j7 + grippger if 6 axis leave 0 #主动机械臂的关节位置

#这四个变量是主手和从手的TCP数据
global qpos_tcp_list
qpos_tcp_list = []

global action_tcp_list
action_tcp_list=[]

# 四元数方式防止变换太大？质检（基本姿态不变）暂时用不到？
global qpos_tcp
qpos_tcp= [0,0,0,0,0,0,0,0] #xyz+ rx,ry,rz,rw

global action_tcp
action_tcp= [0,0,0,0,0,0,0,0] #xyz+ rx,ry,rz,rw

global ForceSensorData_list
ForceSensorData_list = []

global StartCollect
StartCollect=0

global SaveCount
SaveCount=0

global count
count=0

global SleepTime
SleepTime=0.1
global folder_path
folder_path = str(BASE_DIR / 'data')


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1114, 882)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(0, 0, 1071, 731))
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.label_J0_10 = QtWidgets.QLabel(self.tab)
        self.label_J0_10.setGeometry(QtCore.QRect(20, 650, 41, 31))
        self.label_J0_10.setObjectName("label_J0_10")
        self.pushButton_getJ = QtWidgets.QPushButton(self.tab)
        self.pushButton_getJ.setGeometry(QtCore.QRect(780, 240, 121, 51))
        self.pushButton_getJ.setObjectName("pushButton_getJ")
        self.pushButtonFollowStop = QtWidgets.QPushButton(self.tab)
        self.pushButtonFollowStop.setGeometry(QtCore.QRect(170, 170, 131, 51))
        self.pushButtonFollowStop.setObjectName("pushButtonFollowStop")
        self.txtTCPPosA = QtWidgets.QTextEdit(self.tab)
        self.txtTCPPosA.setGeometry(QtCore.QRect(340, 310, 91, 51))
        self.txtTCPPosA.setObjectName("txtTCPPosA")
        self.txtTCPPosC = QtWidgets.QTextEdit(self.tab)
        self.txtTCPPosC.setGeometry(QtCore.QRect(560, 310, 91, 51))
        self.txtTCPPosC.setObjectName("txtTCPPosC")
        self.label_J4 = QtWidgets.QLabel(self.tab)
        self.label_J4.setGeometry(QtCore.QRect(440, 570, 101, 31))
        self.label_J4.setObjectName("label_J4")
        self.label_C = QtWidgets.QLabel(self.tab)
        self.label_C.setGeometry(QtCore.QRect(960, 610, 101, 31))
        self.label_C.setObjectName("label_C")
        self.label_J0_3 = QtWidgets.QLabel(self.tab)
        self.label_J0_3.setGeometry(QtCore.QRect(20, 450, 41, 31))
        self.label_J0_3.setObjectName("label_J0_3")
        self.pushButton_moveL = QtWidgets.QPushButton(self.tab)
        self.pushButton_moveL.setGeometry(QtCore.QRect(920, 310, 121, 51))
        self.pushButton_moveL.setObjectName("pushButton_moveL")
        self.label_J1 = QtWidgets.QLabel(self.tab)
        self.label_J1.setGeometry(QtCore.QRect(440, 450, 101, 31))
        self.label_J1.setObjectName("label_J1")
        self.label_A = QtWidgets.QLabel(self.tab)
        self.label_A.setGeometry(QtCore.QRect(960, 530, 101, 31))
        self.label_A.setObjectName("label_A")
        self.pushButtonCLEARERROR = QtWidgets.QPushButton(self.tab)
        self.pushButtonCLEARERROR.setGeometry(QtCore.QRect(240, 90, 121, 51))
        self.pushButtonCLEARERROR.setObjectName("pushButtonCLEARERROR")
        self.txtTCPPosY = QtWidgets.QTextEdit(self.tab)
        self.txtTCPPosY.setGeometry(QtCore.QRect(120, 310, 91, 51))
        self.txtTCPPosY.setObjectName("txtTCPPosY")
        self.pushButton_moveJ = QtWidgets.QPushButton(self.tab)
        self.pushButton_moveJ.setGeometry(QtCore.QRect(920, 240, 121, 51))
        self.pushButton_moveJ.setObjectName("pushButton_moveJ")
        self.txtJointPos4 = QtWidgets.QTextEdit(self.tab)
        self.txtJointPos4.setGeometry(QtCore.QRect(450, 240, 91, 51))
        self.txtJointPos4.setObjectName("txtJointPos4")
        self.label_J6 = QtWidgets.QLabel(self.tab)
        self.label_J6.setGeometry(QtCore.QRect(440, 650, 101, 31))
        self.label_J6.setObjectName("label_J6")
        self.txtJointPos1 = QtWidgets.QTextEdit(self.tab)
        self.txtJointPos1.setGeometry(QtCore.QRect(120, 240, 91, 51))
        self.txtJointPos1.setObjectName("txtJointPos1")
        self.label_J0_4 = QtWidgets.QLabel(self.tab)
        self.label_J0_4.setGeometry(QtCore.QRect(20, 490, 41, 31))
        self.label_J0_4.setObjectName("label_J0_4")
        self.txtTCPPosEXT = QtWidgets.QTextEdit(self.tab)
        self.txtTCPPosEXT.setGeometry(QtCore.QRect(670, 310, 91, 51))
        self.txtTCPPosEXT.setObjectName("txtTCPPosEXT")
        self.textEdit = QtWidgets.QTextEdit(self.tab)
        self.textEdit.setGeometry(QtCore.QRect(20, 20, 211, 51))
        self.textEdit.setObjectName("textEdit")
        self.pushButton_getL = QtWidgets.QPushButton(self.tab)
        self.pushButton_getL.setGeometry(QtCore.QRect(780, 310, 121, 51))
        self.pushButton_getL.setObjectName("pushButton_getL")
        self.pushButtonON = QtWidgets.QPushButton(self.tab)
        self.pushButtonON.setGeometry(QtCore.QRect(20, 90, 81, 51))
        self.pushButtonON.setObjectName("pushButtonON")
        self.label_J0_6 = QtWidgets.QLabel(self.tab)
        self.label_J0_6.setGeometry(QtCore.QRect(550, 410, 41, 31))
        self.label_J0_6.setObjectName("label_J0_6")
        self.horizontalScrollBarTestValueJ2 = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueJ2.setGeometry(QtCore.QRect(70, 490, 351, 31))
        self.horizontalScrollBarTestValueJ2.setMinimum(-180)
        self.horizontalScrollBarTestValueJ2.setMaximum(180)
        self.horizontalScrollBarTestValueJ2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueJ2.setObjectName("horizontalScrollBarTestValueJ2")
        self.horizontalScrollBarTestValueJ4 = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueJ4.setGeometry(QtCore.QRect(70, 570, 351, 31))
        self.horizontalScrollBarTestValueJ4.setMinimum(-180)
        self.horizontalScrollBarTestValueJ4.setMaximum(180)
        self.horizontalScrollBarTestValueJ4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueJ4.setObjectName("horizontalScrollBarTestValueJ4")
        self.label_Y = QtWidgets.QLabel(self.tab)
        self.label_Y.setGeometry(QtCore.QRect(960, 450, 101, 31))
        self.label_Y.setObjectName("label_Y")
        self.label_J0_14 = QtWidgets.QLabel(self.tab)
        self.label_J0_14.setGeometry(QtCore.QRect(550, 650, 41, 31))
        self.label_J0_14.setObjectName("label_J0_14")
        self.horizontalScrollBarTestValueY = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueY.setGeometry(QtCore.QRect(600, 450, 331, 31))
        self.horizontalScrollBarTestValueY.setMinimum(-180)
        self.horizontalScrollBarTestValueY.setMaximum(180)
        self.horizontalScrollBarTestValueY.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueY.setObjectName("horizontalScrollBarTestValueY")
        self.label_J0_8 = QtWidgets.QLabel(self.tab)
        self.label_J0_8.setGeometry(QtCore.QRect(20, 570, 41, 31))
        self.label_J0_8.setObjectName("label_J0_8")
        self.pushButtonFollowStart = QtWidgets.QPushButton(self.tab)
        self.pushButtonFollowStart.setGeometry(QtCore.QRect(20, 170, 131, 51))
        self.pushButtonFollowStart.setObjectName("pushButtonFollowStart")
        self.txtJointPos2 = QtWidgets.QTextEdit(self.tab)
        self.txtJointPos2.setGeometry(QtCore.QRect(230, 240, 91, 51))
        self.txtJointPos2.setObjectName("txtJointPos2")
        self.label_J3 = QtWidgets.QLabel(self.tab)
        self.label_J3.setGeometry(QtCore.QRect(440, 530, 101, 31))
        self.label_J3.setObjectName("label_J3")
        self.txtJointPos6 = QtWidgets.QTextEdit(self.tab)
        self.txtJointPos6.setGeometry(QtCore.QRect(670, 240, 91, 51))
        self.txtJointPos6.setObjectName("txtJointPos6")
        self.label_Ext = QtWidgets.QLabel(self.tab)
        self.label_Ext.setGeometry(QtCore.QRect(960, 650, 101, 31))
        self.label_Ext.setObjectName("label_Ext")
        self.label_J0_9 = QtWidgets.QLabel(self.tab)
        self.label_J0_9.setGeometry(QtCore.QRect(20, 610, 41, 31))
        self.label_J0_9.setObjectName("label_J0_9")
        self.label_J0_12 = QtWidgets.QLabel(self.tab)
        self.label_J0_12.setGeometry(QtCore.QRect(550, 530, 41, 31))
        self.label_J0_12.setObjectName("label_J0_12")
        self.horizontalScrollBarTestValueJ0 = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueJ0.setGeometry(QtCore.QRect(70, 410, 351, 31))
        self.horizontalScrollBarTestValueJ0.setMinimum(-180)
        self.horizontalScrollBarTestValueJ0.setMaximum(180)
        self.horizontalScrollBarTestValueJ0.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueJ0.setObjectName("horizontalScrollBarTestValueJ0")
        self.label_B = QtWidgets.QLabel(self.tab)
        self.label_B.setGeometry(QtCore.QRect(960, 570, 101, 31))
        self.label_B.setObjectName("label_B")
        self.label_J0_2 = QtWidgets.QLabel(self.tab)
        self.label_J0_2.setGeometry(QtCore.QRect(20, 410, 41, 31))
        self.label_J0_2.setObjectName("label_J0_2")
        self.horizontalScrollBarTestValueZ = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueZ.setGeometry(QtCore.QRect(600, 490, 331, 31))
        self.horizontalScrollBarTestValueZ.setMinimum(-180)
        self.horizontalScrollBarTestValueZ.setMaximum(180)
        self.horizontalScrollBarTestValueZ.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueZ.setObjectName("horizontalScrollBarTestValueZ")
        self.pushButtonCONNECT = QtWidgets.QPushButton(self.tab)
        self.pushButtonCONNECT.setGeometry(QtCore.QRect(240, 20, 121, 51))
        self.pushButtonCONNECT.setObjectName("pushButtonCONNECT")
        self.txtJointPos0 = QtWidgets.QTextEdit(self.tab)
        self.txtJointPos0.setGeometry(QtCore.QRect(20, 240, 91, 51))
        self.txtJointPos0.setObjectName("txtJointPos0")
        self.pushButtonRobotStop = QtWidgets.QPushButton(self.tab)
        self.pushButtonRobotStop.setGeometry(QtCore.QRect(650, 170, 391, 51))
        self.pushButtonRobotStop.setObjectName("pushButtonRobotStop")
        self.pushButtonOFF = QtWidgets.QPushButton(self.tab)
        self.pushButtonOFF.setGeometry(QtCore.QRect(130, 90, 81, 51))
        self.pushButtonOFF.setObjectName("pushButtonOFF")
        self.horizontalScrollBarTestValueJ6 = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueJ6.setGeometry(QtCore.QRect(70, 650, 351, 31))
        self.horizontalScrollBarTestValueJ6.setMinimum(-180)
        self.horizontalScrollBarTestValueJ6.setMaximum(180)
        self.horizontalScrollBarTestValueJ6.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueJ6.setObjectName("horizontalScrollBarTestValueJ6")
        self.txtTCPPosB = QtWidgets.QTextEdit(self.tab)
        self.txtTCPPosB.setGeometry(QtCore.QRect(450, 310, 91, 51))
        self.txtTCPPosB.setObjectName("txtTCPPosB")
        self.label_Z = QtWidgets.QLabel(self.tab)
        self.label_Z.setGeometry(QtCore.QRect(960, 490, 101, 31))
        self.label_Z.setObjectName("label_Z")
        self.horizontalScrollBarTestValueExt = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueExt.setGeometry(QtCore.QRect(600, 650, 331, 31))
        self.horizontalScrollBarTestValueExt.setMinimum(-180)
        self.horizontalScrollBarTestValueExt.setMaximum(180)
        self.horizontalScrollBarTestValueExt.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueExt.setObjectName("horizontalScrollBarTestValueExt")
        self.label_J0_13 = QtWidgets.QLabel(self.tab)
        self.label_J0_13.setGeometry(QtCore.QRect(550, 610, 41, 31))
        self.label_J0_13.setObjectName("label_J0_13")
        self.horizontalScrollBarTestValueJ3 = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueJ3.setGeometry(QtCore.QRect(70, 530, 351, 31))
        self.horizontalScrollBarTestValueJ3.setMinimum(-180)
        self.horizontalScrollBarTestValueJ3.setMaximum(180)
        self.horizontalScrollBarTestValueJ3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueJ3.setObjectName("horizontalScrollBarTestValueJ3")
        self.label_J5 = QtWidgets.QLabel(self.tab)
        self.label_J5.setGeometry(QtCore.QRect(440, 610, 101, 31))
        self.label_J5.setObjectName("label_J5")
        self.label_J0_5 = QtWidgets.QLabel(self.tab)
        self.label_J0_5.setGeometry(QtCore.QRect(20, 530, 41, 31))
        self.label_J0_5.setObjectName("label_J0_5")
        self.label_J2 = QtWidgets.QLabel(self.tab)
        self.label_J2.setGeometry(QtCore.QRect(440, 490, 101, 31))
        self.label_J2.setObjectName("label_J2")
        self.label_J0 = QtWidgets.QLabel(self.tab)
        self.label_J0.setGeometry(QtCore.QRect(440, 410, 101, 31))
        self.label_J0.setObjectName("label_J0")
        self.txtTCPPosX = QtWidgets.QTextEdit(self.tab)
        self.txtTCPPosX.setGeometry(QtCore.QRect(20, 310, 91, 51))
        self.txtTCPPosX.setObjectName("txtTCPPosX")
        self.txtTCPPosZ = QtWidgets.QTextEdit(self.tab)
        self.txtTCPPosZ.setGeometry(QtCore.QRect(230, 310, 91, 51))
        self.txtTCPPosZ.setObjectName("txtTCPPosZ")
        self.horizontalScrollBarTestValueB = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueB.setGeometry(QtCore.QRect(600, 570, 331, 31))
        self.horizontalScrollBarTestValueB.setMinimum(-180)
        self.horizontalScrollBarTestValueB.setMaximum(180)
        self.horizontalScrollBarTestValueB.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueB.setObjectName("horizontalScrollBarTestValueB")
        self.txtJointPos5 = QtWidgets.QTextEdit(self.tab)
        self.txtJointPos5.setGeometry(QtCore.QRect(560, 240, 91, 51))
        self.txtJointPos5.setObjectName("txtJointPos5")
        self.horizontalScrollBarTestValueA = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueA.setGeometry(QtCore.QRect(600, 530, 331, 31))
        self.horizontalScrollBarTestValueA.setMinimum(-180)
        self.horizontalScrollBarTestValueA.setMaximum(180)
        self.horizontalScrollBarTestValueA.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueA.setObjectName("horizontalScrollBarTestValueA")
        self.pushButtonRobotGoHome = QtWidgets.QPushButton(self.tab)
        self.pushButtonRobotGoHome.setGeometry(QtCore.QRect(340, 170, 121, 51))
        self.pushButtonRobotGoHome.setObjectName("pushButtonRobotGoHome")
        self.label_J0_7 = QtWidgets.QLabel(self.tab)
        self.label_J0_7.setGeometry(QtCore.QRect(550, 450, 41, 31))
        self.label_J0_7.setObjectName("label_J0_7")
        self.label_J0_15 = QtWidgets.QLabel(self.tab)
        self.label_J0_15.setGeometry(QtCore.QRect(550, 570, 41, 31))
        self.label_J0_15.setObjectName("label_J0_15")
        self.horizontalScrollBarTestValueC = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueC.setGeometry(QtCore.QRect(600, 610, 331, 31))
        self.horizontalScrollBarTestValueC.setMinimum(-180)
        self.horizontalScrollBarTestValueC.setMaximum(180)
        self.horizontalScrollBarTestValueC.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueC.setObjectName("horizontalScrollBarTestValueC")
        self.label_X = QtWidgets.QLabel(self.tab)
        self.label_X.setGeometry(QtCore.QRect(960, 410, 101, 31))
        self.label_X.setObjectName("label_X")
        self.horizontalScrollBarTestValueJ1 = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueJ1.setGeometry(QtCore.QRect(70, 450, 351, 31))
        self.horizontalScrollBarTestValueJ1.setMinimum(-180)
        self.horizontalScrollBarTestValueJ1.setMaximum(180)
        self.horizontalScrollBarTestValueJ1.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueJ1.setObjectName("horizontalScrollBarTestValueJ1")
        self.pushButtonRobotGoFlat = QtWidgets.QPushButton(self.tab)
        self.pushButtonRobotGoFlat.setGeometry(QtCore.QRect(500, 170, 121, 51))
        self.pushButtonRobotGoFlat.setObjectName("pushButtonRobotGoFlat")
        self.horizontalScrollBarTestValueJ5 = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueJ5.setGeometry(QtCore.QRect(70, 610, 351, 31))
        self.horizontalScrollBarTestValueJ5.setMinimum(-180)
        self.horizontalScrollBarTestValueJ5.setMaximum(180)
        self.horizontalScrollBarTestValueJ5.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueJ5.setObjectName("horizontalScrollBarTestValueJ5")
        self.txtJointPos3 = QtWidgets.QTextEdit(self.tab)
        self.txtJointPos3.setGeometry(QtCore.QRect(340, 240, 91, 51))
        self.txtJointPos3.setObjectName("txtJointPos3")
        self.txtRobotPackage = QtWidgets.QTextEdit(self.tab)
        self.txtRobotPackage.setGeometry(QtCore.QRect(380, 20, 661, 121))
        self.txtRobotPackage.setObjectName("txtRobotPackage")
        self.horizontalScrollBarTestValueX = QtWidgets.QScrollBar(self.tab)
        self.horizontalScrollBarTestValueX.setGeometry(QtCore.QRect(600, 410, 331, 31))
        self.horizontalScrollBarTestValueX.setMinimum(-180)
        self.horizontalScrollBarTestValueX.setMaximum(180)
        self.horizontalScrollBarTestValueX.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarTestValueX.setObjectName("horizontalScrollBarTestValueX")
        self.label_J0_11 = QtWidgets.QLabel(self.tab)
        self.label_J0_11.setGeometry(QtCore.QRect(550, 490, 41, 31))
        self.label_J0_11.setObjectName("label_J0_11")
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.txtRobot1J6 = QtWidgets.QTextEdit(self.tab_2)
        self.txtRobot1J6.setGeometry(QtCore.QRect(680, 70, 91, 51))
        self.txtRobot1J6.setObjectName("txtRobot1J6")
        self.txtRobot1J2 = QtWidgets.QTextEdit(self.tab_2)
        self.txtRobot1J2.setGeometry(QtCore.QRect(240, 70, 91, 51))
        self.txtRobot1J2.setObjectName("txtRobot1J2")
        self.txtRobot1J4 = QtWidgets.QTextEdit(self.tab_2)
        self.txtRobot1J4.setGeometry(QtCore.QRect(460, 70, 91, 51))
        self.txtRobot1J4.setObjectName("txtRobot1J4")
        self.txtRobot1J1 = QtWidgets.QTextEdit(self.tab_2)
        self.txtRobot1J1.setGeometry(QtCore.QRect(130, 70, 91, 51))
        self.txtRobot1J1.setObjectName("txtRobot1J1")
        self.txtRobot1J3 = QtWidgets.QTextEdit(self.tab_2)
        self.txtRobot1J3.setGeometry(QtCore.QRect(350, 70, 91, 51))
        self.txtRobot1J3.setObjectName("txtRobot1J3")
        self.txtRobot1J5 = QtWidgets.QTextEdit(self.tab_2)
        self.txtRobot1J5.setGeometry(QtCore.QRect(570, 70, 91, 51))
        self.txtRobot1J5.setObjectName("txtRobot1J5")
        self.txtRobot1J0 = QtWidgets.QTextEdit(self.tab_2)
        self.txtRobot1J0.setGeometry(QtCore.QRect(30, 70, 91, 51))
        self.txtRobot1J0.setObjectName("txtRobot1J0")
        self.txtGELLO1_J6 = QtWidgets.QTextEdit(self.tab_2)
        self.txtGELLO1_J6.setGeometry(QtCore.QRect(680, 180, 91, 51))
        self.txtGELLO1_J6.setObjectName("txtGELLO1_J6")
        self.txtGELLO1_J2 = QtWidgets.QTextEdit(self.tab_2)
        self.txtGELLO1_J2.setGeometry(QtCore.QRect(240, 180, 91, 51))
        self.txtGELLO1_J2.setObjectName("txtGELLO1_J2")
        self.txtGELLO1_J4 = QtWidgets.QTextEdit(self.tab_2)
        self.txtGELLO1_J4.setGeometry(QtCore.QRect(460, 180, 91, 51))
        self.txtGELLO1_J4.setObjectName("txtGELLO1_J4")
        self.txtGELLO1_J1 = QtWidgets.QTextEdit(self.tab_2)
        self.txtGELLO1_J1.setGeometry(QtCore.QRect(130, 180, 91, 51))
        self.txtGELLO1_J1.setObjectName("txtGELLO1_J1")
        self.txtGELLO1_J3 = QtWidgets.QTextEdit(self.tab_2)
        self.txtGELLO1_J3.setGeometry(QtCore.QRect(350, 180, 91, 51))
        self.txtGELLO1_J3.setObjectName("txtGELLO1_J3")
        self.txtGELLO1_J5 = QtWidgets.QTextEdit(self.tab_2)
        self.txtGELLO1_J5.setGeometry(QtCore.QRect(570, 180, 91, 51))
        self.txtGELLO1_J5.setObjectName("txtGELLO1_J5")
        self.txtGELLO1_J0 = QtWidgets.QTextEdit(self.tab_2)
        self.txtGELLO1_J0.setGeometry(QtCore.QRect(30, 180, 91, 51))
        self.txtGELLO1_J0.setObjectName("txtGELLO1_J0")
        self.label_J0_16 = QtWidgets.QLabel(self.tab_2)
        self.label_J0_16.setGeometry(QtCore.QRect(30, 20, 461, 31))
        self.label_J0_16.setObjectName("label_J0_16")
        self.label_J0_17 = QtWidgets.QLabel(self.tab_2)
        self.label_J0_17.setGeometry(QtCore.QRect(30, 140, 461, 31))
        self.label_J0_17.setObjectName("label_J0_17")
        self.btnGELLO1_FollowStart = QtWidgets.QPushButton(self.tab_2)
        self.btnGELLO1_FollowStart.setGeometry(QtCore.QRect(470, 260, 121, 51))
        self.btnGELLO1_FollowStart.setObjectName("btnGELLO1_FollowStart")
        self.btnGELLO1_BIAS = QtWidgets.QPushButton(self.tab_2)
        self.btnGELLO1_BIAS.setGeometry(QtCore.QRect(30, 370, 121, 51))
        self.btnGELLO1_BIAS.setObjectName("btnGELLO1_BIAS")
        self.label_5 = QtWidgets.QLabel(self.tab_2)
        self.label_5.setGeometry(QtCore.QRect(30, 280, 81, 17))
        self.label_5.setObjectName("label_5")
        self.btnGELLO1_INIT = QtWidgets.QPushButton(self.tab_2)
        self.btnGELLO1_INIT.setGeometry(QtCore.QRect(290, 260, 91, 51))
        self.btnGELLO1_INIT.setObjectName("btnGELLO1_INIT")
        self.txtTargetSerial1 = QtWidgets.QTextEdit(self.tab_2)
        self.txtTargetSerial1.setGeometry(QtCore.QRect(110, 270, 141, 31))
        self.txtTargetSerial1.setObjectName("txtTargetSerial1")
        self.label_6 = QtWidgets.QLabel(self.tab_2)
        self.label_6.setGeometry(QtCore.QRect(30, 320, 221, 17))
        self.label_6.setObjectName("label_6")
        self.txtGELLO1_BIAS_DATA = QtWidgets.QTextEdit(self.tab_2)
        self.txtGELLO1_BIAS_DATA.setGeometry(QtCore.QRect(30, 440, 751, 51))
        self.txtGELLO1_BIAS_DATA.setObjectName("txtGELLO1_BIAS_DATA")
        self.btnGELLO1_FollowStop = QtWidgets.QPushButton(self.tab_2)
        self.btnGELLO1_FollowStop.setGeometry(QtCore.QRect(650, 260, 121, 51))
        self.btnGELLO1_FollowStop.setObjectName("btnGELLO1_FollowStop")
        self.txtGELLO1_JointProtectValue = QtWidgets.QTextEdit(self.tab_2)
        self.txtGELLO1_JointProtectValue.setGeometry(QtCore.QRect(480, 390, 91, 31))
        self.txtGELLO1_JointProtectValue.setObjectName("txtGELLO1_JointProtectValue")
        self.label_7 = QtWidgets.QLabel(self.tab_2)
        self.label_7.setGeometry(QtCore.QRect(470, 340, 351, 20))
        self.label_7.setObjectName("label_7")
        self.tabWidget.addTab(self.tab_2, "")
        self.tab_8 = QtWidgets.QWidget()
        self.tab_8.setObjectName("tab_8")
        self.btnWEIXUE1_FollowStop = QtWidgets.QPushButton(self.tab_8)
        self.btnWEIXUE1_FollowStop.setGeometry(QtCore.QRect(640, 500, 121, 51))
        self.btnWEIXUE1_FollowStop.setObjectName("btnWEIXUE1_FollowStop")
        self.txtRobot1J2_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1J2_WEIXUE.setGeometry(QtCore.QRect(230, 70, 91, 51))
        self.txtRobot1J2_WEIXUE.setObjectName("txtRobot1J2_WEIXUE")
        self.txtWEIXUE_J0 = QtWidgets.QTextEdit(self.tab_8)
        self.txtWEIXUE_J0.setGeometry(QtCore.QRect(20, 180, 91, 51))
        self.txtWEIXUE_J0.setObjectName("txtWEIXUE_J0")
        self.label_J0_26 = QtWidgets.QLabel(self.tab_8)
        self.label_J0_26.setGeometry(QtCore.QRect(20, 20, 461, 31))
        self.label_J0_26.setObjectName("label_J0_26")
        self.label_J0_27 = QtWidgets.QLabel(self.tab_8)
        self.label_J0_27.setGeometry(QtCore.QRect(20, 140, 461, 31))
        self.label_J0_27.setObjectName("label_J0_27")
        self.txtWEIXUE_J2 = QtWidgets.QTextEdit(self.tab_8)
        self.txtWEIXUE_J2.setGeometry(QtCore.QRect(230, 180, 91, 51))
        self.txtWEIXUE_J2.setObjectName("txtWEIXUE_J2")
        self.txtRobot1J4_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1J4_WEIXUE.setGeometry(QtCore.QRect(450, 70, 91, 51))
        self.txtRobot1J4_WEIXUE.setObjectName("txtRobot1J4_WEIXUE")
        self.txtRobot1J1_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1J1_WEIXUE.setGeometry(QtCore.QRect(120, 70, 91, 51))
        self.txtRobot1J1_WEIXUE.setObjectName("txtRobot1J1_WEIXUE")
        self.txtWEIXUE_J3 = QtWidgets.QTextEdit(self.tab_8)
        self.txtWEIXUE_J3.setGeometry(QtCore.QRect(340, 180, 91, 51))
        self.txtWEIXUE_J3.setObjectName("txtWEIXUE_J3")
        self.txtTargetSerial1_2 = QtWidgets.QTextEdit(self.tab_8)
        self.txtTargetSerial1_2.setGeometry(QtCore.QRect(100, 510, 141, 31))
        self.txtTargetSerial1_2.setObjectName("txtTargetSerial1_2")
        self.txtRobot1J6_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1J6_WEIXUE.setGeometry(QtCore.QRect(670, 70, 91, 51))
        self.txtRobot1J6_WEIXUE.setObjectName("txtRobot1J6_WEIXUE")
        self.txtWEIXUE_J1 = QtWidgets.QTextEdit(self.tab_8)
        self.txtWEIXUE_J1.setGeometry(QtCore.QRect(120, 180, 91, 51))
        self.txtWEIXUE_J1.setObjectName("txtWEIXUE_J1")
        self.txtRobot1J0_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1J0_WEIXUE.setGeometry(QtCore.QRect(20, 70, 91, 51))
        self.txtRobot1J0_WEIXUE.setObjectName("txtRobot1J0_WEIXUE")
        self.txtRobot1J3_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1J3_WEIXUE.setGeometry(QtCore.QRect(340, 70, 91, 51))
        self.txtRobot1J3_WEIXUE.setObjectName("txtRobot1J3_WEIXUE")
        self.label_12 = QtWidgets.QLabel(self.tab_8)
        self.label_12.setGeometry(QtCore.QRect(20, 520, 81, 17))
        self.label_12.setObjectName("label_12")
        self.txtRobot1J5_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1J5_WEIXUE.setGeometry(QtCore.QRect(560, 70, 91, 51))
        self.txtRobot1J5_WEIXUE.setObjectName("txtRobot1J5_WEIXUE")
        self.btnWEIXUE1_FollowStart = QtWidgets.QPushButton(self.tab_8)
        self.btnWEIXUE1_FollowStart.setGeometry(QtCore.QRect(460, 500, 121, 51))
        self.btnWEIXUE1_FollowStart.setObjectName("btnWEIXUE1_FollowStart")
        self.btnWEIXUE1_INIT = QtWidgets.QPushButton(self.tab_8)
        self.btnWEIXUE1_INIT.setGeometry(QtCore.QRect(280, 500, 91, 51))
        self.btnWEIXUE1_INIT.setObjectName("btnWEIXUE1_INIT")
        self.label_13 = QtWidgets.QLabel(self.tab_8)
        self.label_13.setGeometry(QtCore.QRect(20, 560, 221, 17))
        self.label_13.setObjectName("label_13")
        self.txtRobot1_Z_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1_Z_WEIXUE.setGeometry(QtCore.QRect(230, 300, 91, 51))
        self.txtRobot1_Z_WEIXUE.setObjectName("txtRobot1_Z_WEIXUE")
        self.txtRobot1_RX_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1_RX_WEIXUE.setGeometry(QtCore.QRect(340, 300, 91, 51))
        self.txtRobot1_RX_WEIXUE.setObjectName("txtRobot1_RX_WEIXUE")
        self.txtRobot1_RZ_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1_RZ_WEIXUE.setGeometry(QtCore.QRect(560, 300, 91, 51))
        self.txtRobot1_RZ_WEIXUE.setObjectName("txtRobot1_RZ_WEIXUE")
        self.txtWEIXUE_Y = QtWidgets.QTextEdit(self.tab_8)
        self.txtWEIXUE_Y.setGeometry(QtCore.QRect(120, 410, 91, 51))
        self.txtWEIXUE_Y.setObjectName("txtWEIXUE_Y")
        self.txtRobot1_Y_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1_Y_WEIXUE.setGeometry(QtCore.QRect(120, 300, 91, 51))
        self.txtRobot1_Y_WEIXUE.setObjectName("txtRobot1_Y_WEIXUE")
        self.txtWEIXUE_Z = QtWidgets.QTextEdit(self.tab_8)
        self.txtWEIXUE_Z.setGeometry(QtCore.QRect(230, 410, 91, 51))
        self.txtWEIXUE_Z.setObjectName("txtWEIXUE_Z")
        self.txtWEIXUE_X = QtWidgets.QTextEdit(self.tab_8)
        self.txtWEIXUE_X.setGeometry(QtCore.QRect(20, 410, 91, 51))
        self.txtWEIXUE_X.setObjectName("txtWEIXUE_X")
        self.txtRobot1_X_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1_X_WEIXUE.setGeometry(QtCore.QRect(20, 300, 91, 51))
        self.txtRobot1_X_WEIXUE.setObjectName("txtRobot1_X_WEIXUE")
        self.label_J0_28 = QtWidgets.QLabel(self.tab_8)
        self.label_J0_28.setGeometry(QtCore.QRect(20, 250, 341, 31))
        self.label_J0_28.setObjectName("label_J0_28")
        self.label_J0_29 = QtWidgets.QLabel(self.tab_8)
        self.label_J0_29.setGeometry(QtCore.QRect(20, 370, 351, 31))
        self.label_J0_29.setObjectName("label_J0_29")
        self.txtRobot1_RY_WEIXUE = QtWidgets.QTextEdit(self.tab_8)
        self.txtRobot1_RY_WEIXUE.setGeometry(QtCore.QRect(450, 300, 91, 51))
        self.txtRobot1_RY_WEIXUE.setObjectName("txtRobot1_RY_WEIXUE")
        self.btnWEIXUE_HOME_ALL = QtWidgets.QPushButton(self.tab_8)
        self.btnWEIXUE_HOME_ALL.setGeometry(QtCore.QRect(720, 300, 271, 141))
        self.btnWEIXUE_HOME_ALL.setObjectName("btnWEIXUE_HOME_ALL")
        self.label_J0_30 = QtWidgets.QLabel(self.tab_8)
        self.label_J0_30.setGeometry(QtCore.QRect(560, 150, 71, 31))
        self.label_J0_30.setObjectName("label_J0_30")
        self.horizontalScrollBarScaleX = QtWidgets.QScrollBar(self.tab_8)
        self.horizontalScrollBarScaleX.setGeometry(QtCore.QRect(640, 150, 351, 31))
        self.horizontalScrollBarScaleX.setMinimum(-180)
        self.horizontalScrollBarScaleX.setMaximum(180)
        self.horizontalScrollBarScaleX.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarScaleX.setObjectName("horizontalScrollBarScaleX")
        self.label_J0_31 = QtWidgets.QLabel(self.tab_8)
        self.label_J0_31.setGeometry(QtCore.QRect(560, 190, 71, 31))
        self.label_J0_31.setObjectName("label_J0_31")
        self.label_J0_32 = QtWidgets.QLabel(self.tab_8)
        self.label_J0_32.setGeometry(QtCore.QRect(560, 230, 71, 31))
        self.label_J0_32.setObjectName("label_J0_32")
        self.horizontalScrollBarScaleY = QtWidgets.QScrollBar(self.tab_8)
        self.horizontalScrollBarScaleY.setGeometry(QtCore.QRect(640, 190, 351, 31))
        self.horizontalScrollBarScaleY.setMinimum(-180)
        self.horizontalScrollBarScaleY.setMaximum(180)
        self.horizontalScrollBarScaleY.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarScaleY.setObjectName("horizontalScrollBarScaleY")
        self.horizontalScrollBarScaleZ = QtWidgets.QScrollBar(self.tab_8)
        self.horizontalScrollBarScaleZ.setGeometry(QtCore.QRect(640, 230, 351, 31))
        self.horizontalScrollBarScaleZ.setMinimum(-180)
        self.horizontalScrollBarScaleZ.setMaximum(180)
        self.horizontalScrollBarScaleZ.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalScrollBarScaleZ.setObjectName("horizontalScrollBarScaleZ")
        self.tabWidget.addTab(self.tab_8, "")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.btnGripper1_INIT = QtWidgets.QPushButton(self.tab_3)
        self.btnGripper1_INIT.setGeometry(QtCore.QRect(290, 30, 91, 51))
        self.btnGripper1_INIT.setObjectName("btnGripper1_INIT")
        self.label_8 = QtWidgets.QLabel(self.tab_3)
        self.label_8.setGeometry(QtCore.QRect(30, 50, 81, 17))
        self.label_8.setObjectName("label_8")
        self.txtTargetSerialGripper = QtWidgets.QTextEdit(self.tab_3)
        self.txtTargetSerialGripper.setGeometry(QtCore.QRect(110, 40, 141, 31))
        self.txtTargetSerialGripper.setObjectName("txtTargetSerialGripper")
        self.btnGripper1_Open = QtWidgets.QPushButton(self.tab_3)
        self.btnGripper1_Open.setGeometry(QtCore.QRect(450, 30, 91, 51))
        self.btnGripper1_Open.setObjectName("btnGripper1_Open")
        self.btnGripper1_Close = QtWidgets.QPushButton(self.tab_3)
        self.btnGripper1_Close.setGeometry(QtCore.QRect(570, 30, 91, 51))
        self.btnGripper1_Close.setObjectName("btnGripper1_Close")
        self.txtGripper1_TargetJ0 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetJ0.setGeometry(QtCore.QRect(130, 490, 91, 51))
        self.txtGripper1_TargetJ0.setObjectName("txtGripper1_TargetJ0")
        self.txtGripper1_Torque = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_Torque.setGeometry(QtCore.QRect(210, 280, 601, 51))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Condensed")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.txtGripper1_Torque.setFont(font)
        self.txtGripper1_Torque.setObjectName("txtGripper1_Torque")
        self.txtGripper1_TargetJ4 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetJ4.setGeometry(QtCore.QRect(560, 490, 91, 51))
        self.txtGripper1_TargetJ4.setObjectName("txtGripper1_TargetJ4")
        self.txtGripper1_Error = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_Error.setGeometry(QtCore.QRect(210, 340, 601, 51))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Condensed")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.txtGripper1_Error.setFont(font)
        self.txtGripper1_Error.setObjectName("txtGripper1_Error")
        self.txtGripper1_TargetJ5 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetJ5.setGeometry(QtCore.QRect(670, 490, 91, 51))
        self.txtGripper1_TargetJ5.setObjectName("txtGripper1_TargetJ5")
        self.label_J0_18 = QtWidgets.QLabel(self.tab_3)
        self.label_J0_18.setGeometry(QtCore.QRect(20, 500, 101, 31))
        self.label_J0_18.setObjectName("label_J0_18")
        self.txtGripper1_Pos = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_Pos.setGeometry(QtCore.QRect(210, 160, 601, 51))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Condensed")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.txtGripper1_Pos.setFont(font)
        self.txtGripper1_Pos.setObjectName("txtGripper1_Pos")
        self.txtGripper1_TargetJ6 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetJ6.setGeometry(QtCore.QRect(780, 490, 91, 51))
        self.txtGripper1_TargetJ6.setObjectName("txtGripper1_TargetJ6")
        self.txtGripper1_TargetJ2 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetJ2.setGeometry(QtCore.QRect(340, 490, 91, 51))
        self.txtGripper1_TargetJ2.setObjectName("txtGripper1_TargetJ2")
        self.txtGripper1_TargetJ3 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetJ3.setGeometry(QtCore.QRect(450, 490, 91, 51))
        self.txtGripper1_TargetJ3.setObjectName("txtGripper1_TargetJ3")
        self.label_J0_19 = QtWidgets.QLabel(self.tab_3)
        self.label_J0_19.setGeometry(QtCore.QRect(30, 170, 151, 31))
        self.label_J0_19.setObjectName("label_J0_19")
        self.txtGripper1_TargetJ1 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetJ1.setGeometry(QtCore.QRect(230, 490, 91, 51))
        self.txtGripper1_TargetJ1.setObjectName("txtGripper1_TargetJ1")
        self.txtGripper1_Vel = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_Vel.setGeometry(QtCore.QRect(210, 220, 601, 51))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Condensed")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.txtGripper1_Vel.setFont(font)
        self.txtGripper1_Vel.setObjectName("txtGripper1_Vel")
        self.btnGripper1_SETPOS = QtWidgets.QPushButton(self.tab_3)
        self.btnGripper1_SETPOS.setGeometry(QtCore.QRect(900, 560, 121, 51))
        self.btnGripper1_SETPOS.setObjectName("btnGripper1_SETPOS")
        self.btnGripper1_GETPOS = QtWidgets.QPushButton(self.tab_3)
        self.btnGripper1_GETPOS.setGeometry(QtCore.QRect(900, 490, 121, 51))
        self.btnGripper1_GETPOS.setObjectName("btnGripper1_GETPOS")
        self.txtGripper1_TargetVel5 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetVel5.setGeometry(QtCore.QRect(670, 560, 91, 51))
        self.txtGripper1_TargetVel5.setObjectName("txtGripper1_TargetVel5")
        self.txtGripper1_TargetVel6 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetVel6.setGeometry(QtCore.QRect(780, 560, 91, 51))
        self.txtGripper1_TargetVel6.setObjectName("txtGripper1_TargetVel6")
        self.txtGripper1_TargetVel4 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetVel4.setGeometry(QtCore.QRect(560, 560, 91, 51))
        self.txtGripper1_TargetVel4.setObjectName("txtGripper1_TargetVel4")
        self.txtGripper1_TargetVel0 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetVel0.setGeometry(QtCore.QRect(130, 560, 91, 51))
        self.txtGripper1_TargetVel0.setObjectName("txtGripper1_TargetVel0")
        self.txtGripper1_TargetVel1 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetVel1.setGeometry(QtCore.QRect(230, 560, 91, 51))
        self.txtGripper1_TargetVel1.setObjectName("txtGripper1_TargetVel1")
        self.txtGripper1_TargetVel2 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetVel2.setGeometry(QtCore.QRect(340, 560, 91, 51))
        self.txtGripper1_TargetVel2.setObjectName("txtGripper1_TargetVel2")
        self.txtGripper1_TargetVel3 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetVel3.setGeometry(QtCore.QRect(450, 560, 91, 51))
        self.txtGripper1_TargetVel3.setObjectName("txtGripper1_TargetVel3")
        self.txtGripper1_TargetTorque5 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetTorque5.setGeometry(QtCore.QRect(670, 630, 91, 51))
        self.txtGripper1_TargetTorque5.setObjectName("txtGripper1_TargetTorque5")
        self.txtGripper1_TargetTorque6 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetTorque6.setGeometry(QtCore.QRect(780, 630, 91, 51))
        self.txtGripper1_TargetTorque6.setObjectName("txtGripper1_TargetTorque6")
        self.txtGripper1_TargetTorque4 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetTorque4.setGeometry(QtCore.QRect(560, 630, 91, 51))
        self.txtGripper1_TargetTorque4.setObjectName("txtGripper1_TargetTorque4")
        self.txtGripper1_TargetTorque0 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetTorque0.setGeometry(QtCore.QRect(130, 630, 91, 51))
        self.txtGripper1_TargetTorque0.setObjectName("txtGripper1_TargetTorque0")
        self.txtGripper1_TargetTorque1 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetTorque1.setGeometry(QtCore.QRect(230, 630, 91, 51))
        self.txtGripper1_TargetTorque1.setObjectName("txtGripper1_TargetTorque1")
        self.txtGripper1_TargetTorque2 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetTorque2.setGeometry(QtCore.QRect(340, 630, 91, 51))
        self.txtGripper1_TargetTorque2.setObjectName("txtGripper1_TargetTorque2")
        self.txtGripper1_TargetTorque3 = QtWidgets.QTextEdit(self.tab_3)
        self.txtGripper1_TargetTorque3.setGeometry(QtCore.QRect(450, 630, 91, 51))
        self.txtGripper1_TargetTorque3.setObjectName("txtGripper1_TargetTorque3")
        self.btnGripper1_GETPOS_2 = QtWidgets.QPushButton(self.tab_3)
        self.btnGripper1_GETPOS_2.setGeometry(QtCore.QRect(840, 170, 121, 51))
        self.btnGripper1_GETPOS_2.setObjectName("btnGripper1_GETPOS_2")
        self.btnGripper1_STOP = QtWidgets.QPushButton(self.tab_3)
        self.btnGripper1_STOP.setGeometry(QtCore.QRect(900, 630, 121, 51))
        self.btnGripper1_STOP.setObjectName("btnGripper1_STOP")
        self.btnGripper1_OFF = QtWidgets.QPushButton(self.tab_3)
        self.btnGripper1_OFF.setGeometry(QtCore.QRect(850, 100, 91, 51))
        self.btnGripper1_OFF.setObjectName("btnGripper1_OFF")
        self.btnGripper1_ON = QtWidgets.QPushButton(self.tab_3)
        self.btnGripper1_ON.setGeometry(QtCore.QRect(850, 30, 91, 51))
        self.btnGripper1_ON.setObjectName("btnGripper1_ON")
        self.label_J0_20 = QtWidgets.QLabel(self.tab_3)
        self.label_J0_20.setGeometry(QtCore.QRect(30, 230, 151, 31))
        self.label_J0_20.setObjectName("label_J0_20")
        self.label_J0_21 = QtWidgets.QLabel(self.tab_3)
        self.label_J0_21.setGeometry(QtCore.QRect(30, 290, 171, 31))
        self.label_J0_21.setObjectName("label_J0_21")
        self.label_J0_22 = QtWidgets.QLabel(self.tab_3)
        self.label_J0_22.setGeometry(QtCore.QRect(30, 350, 161, 31))
        self.label_J0_22.setObjectName("label_J0_22")
        self.label_J0_23 = QtWidgets.QLabel(self.tab_3)
        self.label_J0_23.setGeometry(QtCore.QRect(20, 570, 81, 31))
        self.label_J0_23.setObjectName("label_J0_23")
        self.label_J0_24 = QtWidgets.QLabel(self.tab_3)
        self.label_J0_24.setGeometry(QtCore.QRect(20, 640, 101, 31))
        self.label_J0_24.setObjectName("label_J0_24")
        self.tabWidget.addTab(self.tab_3, "")
        self.tab_4 = QtWidgets.QWidget()
        self.tab_4.setObjectName("tab_4")
        self.tabWidget_2 = QtWidgets.QTabWidget(self.tab_4)
        self.tabWidget_2.setGeometry(QtCore.QRect(10, 90, 1041, 581))
        self.tabWidget_2.setObjectName("tabWidget_2")
        self.tab_5 = QtWidgets.QWidget()
        self.tab_5.setObjectName("tab_5")
        self.D435_Pic_Color = QtWidgets.QLabel(self.tab_5)
        self.D435_Pic_Color.setGeometry(QtCore.QRect(10, 20, 640, 480))
        self.D435_Pic_Color.setObjectName("D435_Pic_Color")
        self.tabWidget_2.addTab(self.tab_5, "")
        self.tab_6 = QtWidgets.QWidget()
        self.tab_6.setObjectName("tab_6")
        self.D435_Pic_Depth = QtWidgets.QLabel(self.tab_6)
        self.D435_Pic_Depth.setGeometry(QtCore.QRect(20, 20, 640, 480))
        self.D435_Pic_Depth.setObjectName("D435_Pic_Depth")
        self.tabWidget_2.addTab(self.tab_6, "")
        self.btnD435_1_Stop = QtWidgets.QPushButton(self.tab_4)
        self.btnD435_1_Stop.setGeometry(QtCore.QRect(150, 20, 121, 51))
        self.btnD435_1_Stop.setObjectName("btnD435_1_Stop")
        self.btnD435_1_Cut = QtWidgets.QPushButton(self.tab_4)
        self.btnD435_1_Cut.setGeometry(QtCore.QRect(290, 20, 121, 51))
        self.btnD435_1_Cut.setObjectName("btnD435_1_Cut")
        self.btnD435_1_Start = QtWidgets.QPushButton(self.tab_4)
        self.btnD435_1_Start.setGeometry(QtCore.QRect(10, 20, 121, 51))
        self.btnD435_1_Start.setObjectName("btnD435_1_Start")
        self.tabWidget.addTab(self.tab_4, "")
        self.tab_7 = QtWidgets.QWidget()
        self.tab_7.setObjectName("tab_7")
        self.txtForceSensorValue = QtWidgets.QTextEdit(self.tab_7)
        self.txtForceSensorValue.setGeometry(QtCore.QRect(10, 150, 1001, 61))
        self.txtForceSensorValue.setObjectName("txtForceSensorValue")
        self.label_J0_25 = QtWidgets.QLabel(self.tab_7)
        self.label_J0_25.setGeometry(QtCore.QRect(20, 110, 211, 31))
        self.label_J0_25.setObjectName("label_J0_25")
        self.label_11 = QtWidgets.QLabel(self.tab_7)
        self.label_11.setGeometry(QtCore.QRect(20, 50, 81, 17))
        self.label_11.setObjectName("label_11")
        self.btnForceSensor_INIT = QtWidgets.QPushButton(self.tab_7)
        self.btnForceSensor_INIT.setGeometry(QtCore.QRect(280, 30, 91, 51))
        self.btnForceSensor_INIT.setObjectName("btnForceSensor_INIT")
        self.txtTargetSerialForceSensor = QtWidgets.QTextEdit(self.tab_7)
        self.txtTargetSerialForceSensor.setGeometry(QtCore.QRect(100, 40, 141, 31))
        self.txtTargetSerialForceSensor.setObjectName("txtTargetSerialForceSensor")
        self.tabWidget.addTab(self.tab_7, "")
        self.btnCollectStart = QtWidgets.QPushButton(self.centralwidget)
        self.btnCollectStart.setGeometry(QtCore.QRect(10, 750, 121, 71))
        self.btnCollectStart.setObjectName("btnCollectStart")
        self.btnCollectStop = QtWidgets.QPushButton(self.centralwidget)
        self.btnCollectStop.setGeometry(QtCore.QRect(150, 750, 121, 71))
        self.btnCollectStop.setObjectName("btnCollectStop")
        self.label_State = QtWidgets.QLabel(self.centralwidget)
        self.label_State.setGeometry(QtCore.QRect(360, 800, 81, 17))
        self.label_State.setObjectName("label_State")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(300, 760, 51, 17))
        self.label_3.setObjectName("label_3")
        self.label_RecordCount = QtWidgets.QLabel(self.centralwidget)
        self.label_RecordCount.setGeometry(QtCore.QRect(360, 760, 81, 17))
        self.label_RecordCount.setObjectName("label_RecordCount")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(300, 800, 51, 17))
        self.label_4.setObjectName("label_4")
        self.label_9 = QtWidgets.QLabel(self.centralwidget)
        self.label_9.setGeometry(QtCore.QRect(590, 760, 101, 17))
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_10.setGeometry(QtCore.QRect(590, 800, 71, 17))
        self.label_10.setObjectName("label_10")
        self.txtCollectTimeSpan = QtWidgets.QTextEdit(self.centralwidget)
        self.txtCollectTimeSpan.setGeometry(QtCore.QRect(680, 750, 111, 31))
        self.txtCollectTimeSpan.setObjectName("txtCollectTimeSpan")
        self.txtFileSavePath = QtWidgets.QTextEdit(self.centralwidget)
        self.txtFileSavePath.setGeometry(QtCore.QRect(680, 790, 271, 31))
        self.txtFileSavePath.setObjectName("txtFileSavePath")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1114, 28))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(2)
        self.tabWidget_2.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_J0_10.setText(_translate("MainWindow", "J6"))
        self.pushButton_getJ.setText(_translate("MainWindow", "GET_J"))
        self.pushButtonFollowStop.setText(_translate("MainWindow", "TEST-FollowStop"))
        self.label_J4.setText(_translate("MainWindow", "TextLabel"))
        self.label_C.setText(_translate("MainWindow", "TextLabel"))
        self.label_J0_3.setText(_translate("MainWindow", "J1"))
        self.pushButton_moveL.setText(_translate("MainWindow", "MOVE_L"))
        self.label_J1.setText(_translate("MainWindow", "TextLabel"))
        self.label_A.setText(_translate("MainWindow", "TextLabel"))
        self.pushButtonCLEARERROR.setText(_translate("MainWindow", "CLEARERROR"))
        self.pushButton_moveJ.setText(_translate("MainWindow", "MOVE_J"))
        self.label_J6.setText(_translate("MainWindow", "TextLabel"))
        self.label_J0_4.setText(_translate("MainWindow", "J2"))
        self.textEdit.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; background-color:#2b2b2b;\"><span style=\" font-family:\'JetBrains Mono,monospace\'; font-size:18pt; color:#6a8759;\">192.168.3.15</span></p></body></html>"))
        self.pushButton_getL.setText(_translate("MainWindow", "GET_L"))
        self.pushButtonON.setText(_translate("MainWindow", "ON"))
        self.label_J0_6.setText(_translate("MainWindow", "X"))
        self.label_Y.setText(_translate("MainWindow", "TextLabel"))
        self.label_J0_14.setText(_translate("MainWindow", "Ext"))
        self.label_J0_8.setText(_translate("MainWindow", "J4"))
        self.pushButtonFollowStart.setText(_translate("MainWindow", "TEST-FollowStart"))
        self.label_J3.setText(_translate("MainWindow", "TextLabel"))
        self.label_Ext.setText(_translate("MainWindow", "TextLabel"))
        self.label_J0_9.setText(_translate("MainWindow", "J5"))
        self.label_J0_12.setText(_translate("MainWindow", "A"))
        self.label_B.setText(_translate("MainWindow", "TextLabel"))
        self.label_J0_2.setText(_translate("MainWindow", "J0"))
        self.pushButtonCONNECT.setText(_translate("MainWindow", "CONNECT"))
        self.pushButtonRobotStop.setText(_translate("MainWindow", "STOP"))
        self.pushButtonOFF.setText(_translate("MainWindow", "OFF"))
        self.label_Z.setText(_translate("MainWindow", "TextLabel"))
        self.label_J0_13.setText(_translate("MainWindow", "C"))
        self.label_J5.setText(_translate("MainWindow", "TextLabel"))
        self.label_J0_5.setText(_translate("MainWindow", "J3"))
        self.label_J2.setText(_translate("MainWindow", "TextLabel"))
        self.label_J0.setText(_translate("MainWindow", "TextLabel"))
        self.pushButtonRobotGoHome.setText(_translate("MainWindow", "HOME"))
        self.label_J0_7.setText(_translate("MainWindow", "Y"))
        self.label_J0_15.setText(_translate("MainWindow", "B"))
        self.label_X.setText(_translate("MainWindow", "TextLabel"))
        self.pushButtonRobotGoFlat.setText(_translate("MainWindow", "FLAT"))
        self.label_J0_11.setText(_translate("MainWindow", "Z"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Robot1"))
        self.label_J0_16.setText(_translate("MainWindow", "Robot CurrentPos(must in first page connect robot)"))
        self.label_J0_17.setText(_translate("MainWindow", "Gello CurrentPos(must set right serial to connect)"))
        self.btnGELLO1_FollowStart.setText(_translate("MainWindow", "FollowStart"))
        self.btnGELLO1_BIAS.setText(_translate("MainWindow", "BIAS"))
        self.label_5.setText(_translate("MainWindow", "SerialPort"))
        self.btnGELLO1_INIT.setText(_translate("MainWindow", "INIT"))
        self.txtTargetSerial1.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; background-color:#2b2b2b;\"><span style=\" font-family:\'JetBrains Mono,monospace\'; font-size:9.8pt; color:#6a8759;\">/dev/ttyUSB0</span></p></body></html>"))
        self.label_6.setText(_translate("MainWindow", "To Scan Serial : ls /dev/ttyUSB*"))
        self.btnGELLO1_FollowStop.setText(_translate("MainWindow", "FollowStop"))
        self.txtGELLO1_JointProtectValue.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">90</p></body></html>"))
        self.label_7.setText(_translate("MainWindow", "Joint Follow Protect Value(set 999 to ignore):"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "GELLO1"))
        self.btnWEIXUE1_FollowStop.setText(_translate("MainWindow", "FollowStop"))
        self.label_J0_26.setText(_translate("MainWindow", "Robot JointPos(must in first page connect robot)"))
        self.label_J0_27.setText(_translate("MainWindow", "WEIXUE JointPos(must set right serial to connect)"))
        self.txtTargetSerial1_2.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; background-color:#2b2b2b;\"><span style=\" font-family:\'JetBrains Mono,monospace\'; font-size:9.8pt; color:#6a8759;\">/dev/ttyUSB0</span></p></body></html>"))
        self.label_12.setText(_translate("MainWindow", "SerialPort"))
        self.btnWEIXUE1_FollowStart.setText(_translate("MainWindow", "FollowStart"))
        self.btnWEIXUE1_INIT.setText(_translate("MainWindow", "INIT"))
        self.label_13.setText(_translate("MainWindow", "To Scan Serial : ls /dev/ttyUSB*"))
        self.label_J0_28.setText(_translate("MainWindow", "Robot TCPPos(must in first page connect robot)"))
        self.label_J0_29.setText(_translate("MainWindow", "WEIXUE TCPPos(must set right serial to connect)"))
        self.btnWEIXUE_HOME_ALL.setText(_translate("MainWindow", "ALL-HOME"))
        self.label_J0_30.setText(_translate("MainWindow", "Scale_X"))
        self.label_J0_31.setText(_translate("MainWindow", "Scale_Y"))
        self.label_J0_32.setText(_translate("MainWindow", "Scale_Z"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_8), _translate("MainWindow", "WEIXUE1"))
        self.btnGripper1_INIT.setText(_translate("MainWindow", "INIT"))
        self.label_8.setText(_translate("MainWindow", "SerialPort"))
        self.txtTargetSerialGripper.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; background-color:#2b2b2b;\"><span style=\" font-family:\'JetBrains Mono,monospace\'; font-size:9.8pt; color:#6a8759;\">/dev/ttyUSB1</span></p></body></html>"))
        self.btnGripper1_Open.setText(_translate("MainWindow", "OPEN"))
        self.btnGripper1_Close.setText(_translate("MainWindow", "CLOSE"))
        self.txtGripper1_TargetJ0.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetJ4.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetJ5.setPlaceholderText(_translate("MainWindow", "100"))
        self.label_J0_18.setText(_translate("MainWindow", "Target Pos"))
        self.txtGripper1_TargetJ6.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetJ2.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetJ3.setPlaceholderText(_translate("MainWindow", "100"))
        self.label_J0_19.setText(_translate("MainWindow", "Gripper1 Current Pos"))
        self.txtGripper1_TargetJ1.setPlaceholderText(_translate("MainWindow", "100"))
        self.btnGripper1_SETPOS.setText(_translate("MainWindow", "MOVE"))
        self.btnGripper1_GETPOS.setText(_translate("MainWindow", "GET"))
        self.txtGripper1_TargetVel5.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetVel6.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetVel4.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetVel0.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetVel1.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetVel2.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetVel3.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetTorque5.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetTorque6.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetTorque4.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetTorque0.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetTorque1.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetTorque2.setPlaceholderText(_translate("MainWindow", "100"))
        self.txtGripper1_TargetTorque3.setPlaceholderText(_translate("MainWindow", "100"))
        self.btnGripper1_GETPOS_2.setText(_translate("MainWindow", "ClearError"))
        self.btnGripper1_STOP.setText(_translate("MainWindow", "STOP"))
        self.btnGripper1_OFF.setText(_translate("MainWindow", "OFF"))
        self.btnGripper1_ON.setText(_translate("MainWindow", "ON"))
        self.label_J0_20.setText(_translate("MainWindow", "Gripper1 Current Vel"))
        self.label_J0_21.setText(_translate("MainWindow", "Gripper1 Current Torque"))
        self.label_J0_22.setText(_translate("MainWindow", "Gripper1 Current  Error"))
        self.label_J0_23.setText(_translate("MainWindow", "Target Vel"))
        self.label_J0_24.setText(_translate("MainWindow", "Target Torque"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("MainWindow", "Gripper1"))
        self.D435_Pic_Color.setText(_translate("MainWindow", "TextLabel"))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_5), _translate("MainWindow", "Color1"))
        self.D435_Pic_Depth.setText(_translate("MainWindow", "TextLabel"))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_6), _translate("MainWindow", "Depth1"))
        self.btnD435_1_Stop.setText(_translate("MainWindow", "STOP"))
        self.btnD435_1_Cut.setText(_translate("MainWindow", "CUT"))
        self.btnD435_1_Start.setText(_translate("MainWindow", "START"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), _translate("MainWindow", "CAMERA1"))
        self.label_J0_25.setText(_translate("MainWindow", "CurrentForceSensorValue"))
        self.label_11.setText(_translate("MainWindow", "SerialPort"))
        self.btnForceSensor_INIT.setText(_translate("MainWindow", "INIT"))
        self.txtTargetSerialForceSensor.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; background-color:#2b2b2b;\"><span style=\" font-family:\'JetBrains Mono,monospace\'; font-size:9.8pt; color:#6a8759;\">/dev/ttyUSB2</span></p></body></html>"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_7), _translate("MainWindow", "ForceSensor1"))
        self.btnCollectStart.setText(_translate("MainWindow", "Collect-Start"))
        self.btnCollectStop.setText(_translate("MainWindow", "Collect-Stop"))
        self.label_State.setText(_translate("MainWindow", "?"))
        self.label_3.setText(_translate("MainWindow", "Count"))
        self.label_RecordCount.setText(_translate("MainWindow", "?"))
        self.label_4.setText(_translate("MainWindow", "Total"))
        self.label_9.setText(_translate("MainWindow", "采样间隔"))
        self.label_10.setText(_translate("MainWindow", "保存路径"))
        self.txtCollectTimeSpan.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">0.1</p></body></html>"))
        self.txtFileSavePath.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; background-color:#ffffff;\"><span style=\" font-family:\'JetBrains Mono,monospace\'; font-size:9.8pt; font-weight:600; color:#008080;\">/root/AAA</span></p></body></html>"))



class mythread_WEIXUE_MasterHand(QThread):

    def __init__(self):
        super(mythread_WEIXUE_MasterHand, self).__init__()

    def run(self):
        # 多线程里面执行子类的死循环
        print("mythread_WEIXUE_MasterHand !!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        if MasterHand_WEIXUE is not None:
            MasterHand_WEIXUE.mainloop()


class mythread_ForceSensor(QThread):

    def __init__(self):
        super(mythread_ForceSensor, self).__init__()

    def run(self):
        # 多线程里面执行子类的死循环
        print("mythread_ForceSensor !!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        Sensor_Robotiq.mainloop()

class MyMainForm(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MyMainForm, self).__init__(parent)
        self.setupUi(self)
        workspace_font = self.font()
        workspace_font.setPointSize(11)
        self.setFont(workspace_font)
        self.setWindowTitle("GELLO · CR3A / O6 [ServoJ 实验副本]")
        self.socketFd = -1
        self.socketFd_7000 = -1
        self.nrc_adapter = None
        self.cr3a_device = None
        self._teleop_async_threads = {}
        self._nrc_message_last = {}

        # 机器人页面测试
        self.pushButtonCONNECT.clicked.connect(self.RobotCONNECT)
        self.pushButtonON.clicked.connect(self.RobotPowerON)
        self.pushButtonOFF.clicked.connect(self.RobotPowerOFF)
        self.pushButtonCLEARERROR.clicked.connect(self.RobotClearError)

        self.pushButtonFollowStart.clicked.connect(self.RobotTrackStart)
        self.pushButtonFollowStop.clicked.connect(self.RobotTrackStop)
        self.pushButtonRobotStop.clicked.connect(self.EmergencyStop)

        self.pushButtonRobotGoHome.clicked.connect(self.RobotGoHome)
        self.pushButtonRobotGoFlat.clicked.connect(self.RobotGoFlat)

        self.pushButton_getJ.clicked.connect(self.RobotGetJ)
        self.pushButton_moveJ.clicked.connect(self.RobotMoveJ)

        self.pushButton_getL.clicked.connect(self.RobotGetL)
        self.pushButton_moveL.clicked.connect(self.RobotMoveL)

        self.horizontalScrollBarTestValueJ0.valueChanged.connect(self.Track_ActionJ0)
        self.horizontalScrollBarTestValueJ1.valueChanged.connect(self.Track_ActionJ1)
        self.horizontalScrollBarTestValueJ2.valueChanged.connect(self.Track_ActionJ2)
        self.horizontalScrollBarTestValueJ3.valueChanged.connect(self.Track_ActionJ3)
        self.horizontalScrollBarTestValueJ4.valueChanged.connect(self.Track_ActionJ4)
        self.horizontalScrollBarTestValueJ5.valueChanged.connect(self.Track_ActionJ5)
        self.horizontalScrollBarTestValueJ6.valueChanged.connect(self.Track_ActionJ6)

        self.horizontalScrollBarTestValueX.valueChanged.connect(self.Track_ActionX)
        self.horizontalScrollBarTestValueY.valueChanged.connect(self.Track_ActionY)
        self.horizontalScrollBarTestValueZ.valueChanged.connect(self.Track_ActionZ)
        self.horizontalScrollBarTestValueA.valueChanged.connect(self.Track_ActionA)
        self.horizontalScrollBarTestValueB.valueChanged.connect(self.Track_ActionB)
        self.horizontalScrollBarTestValueC.valueChanged.connect(self.Track_ActionC)
        # 旧滚动条会直接下发 servoJ，与 XYZ 遥操竞争；保留显示但禁用输入。
        for slider in (
            self.horizontalScrollBarTestValueJ0,
            self.horizontalScrollBarTestValueJ1,
            self.horizontalScrollBarTestValueJ2,
            self.horizontalScrollBarTestValueJ3,
            self.horizontalScrollBarTestValueJ4,
            self.horizontalScrollBarTestValueJ5,
            self.horizontalScrollBarTestValueJ6,
            self.horizontalScrollBarTestValueX,
            self.horizontalScrollBarTestValueY,
            self.horizontalScrollBarTestValueZ,
            self.horizontalScrollBarTestValueA,
            self.horizontalScrollBarTestValueB,
            self.horizontalScrollBarTestValueC,
        ):
            slider.setEnabled(False)
            slider.setToolTip("已禁用：请使用‘RoArm 遥操’页的 XYZ 增量跟随")

        # 机器人数据
        self.robot1_connected = False
        self.robot1_message = ''
        self.robot1_joint_pos = [0,0,0,0,0,0,0]
        self.robot1_tcp_pos = [0,0,0,0,0,0,0]

        # RoArm-M3-Pro / CR5 / O6 新遥操作运行时。硬件 I/O 全部在后台线程中。
        self._setup_roarm_teleop()
        self._setup_responsive_main_layout()

        # GELLO主手页面测试
        self.btnGELLO1_INIT.clicked.connect(self.GELLO1_INIT)
        self.btnGELLO1_BIAS.clicked.connect(self.GELLO1_BIAS)
        self.btnGELLO1_FollowStart.clicked.connect(self.GELLO1_FollowStart)
        self.btnGELLO1_FollowStop.clicked.connect(self.GELLO1_FollowStop)

        # 遥操作的差异值（机械臂位置不变，发送的时候让主手从等同于当前位置即可）
        #self.robot2_joint_bias = [0, 0, 0, 0, 0, 0, 0]
        #self.robot2_joint_bias = [280.66,-177.78,177.55,187.62,-179.73,12.93,207.16]
        #self.robot2_joint_bias = [280.66, -177.78, 177.55, 187.62, (-179.73+360),(12.93+360), 207.16]
        self.robot2_joint_bias = [179, (-177+360), -5, (-176+360), 357, 449, 207.16]

        self.robot2_if_joint_track = False
        self.robot2_joint_tracking_diff_protect= 90 # 超过多少度就停止跟随

        # self.robot2_tcp_bias = [0, 0, 0, 0, 0, 0, 0]
        self.robot2_tcp_bias = [0, 0, 0, 0, 0, 0, 0]
        self.robot2_if_TCP_track = False
        self.robot3__tracking_diff_protect = 300  # 超过多少mm就停止跟随

        # WEIXUE主手页面测试
        self.btnWEIXUE1_INIT.clicked.connect(self.WEIXUE1_INIT)
        self.btnWEIXUE_HOME_ALL.clicked.connect(self.WEIXUE_HOME_ALL)
        self.btnWEIXUE1_FollowStart.clicked.connect(self.WEIXUE1_FollowStart)
        self.btnWEIXUE1_FollowStop.clicked.connect(self.WEIXUE1_FollowStop)

        # 设置定时器，每隔30ms刷新一次画面
        # The GELLO workspace does not run legacy WEIXUE polling.

        # 灵心巧手控制页面
        self.hand_joint = None
        self.hand_type = None
        self.api = None

        self.btnGripper1_INIT.clicked.connect(self.Gripper1_INIT)
        self.btnGripper1_Open.clicked.connect(self.Gripper1_Open)
        self.btnGripper1_Close.clicked.connect(self.Gripper1_Close)
        self.btnGripper1_GETPOS.clicked.connect(self.Gripper1_GetPos)
        self.btnGripper1_SETPOS.clicked.connect(self.Gripper1_SetPos)

        # D435 视觉页面
        # CAMERA1 固定为腕部相机；CAMERA2 固定为基座相机并生成 ROI。
        dataset_camera_cfg = self.teleop_store.data["dataset"]
        self.wrist_camera_serial = str(dataset_camera_cfg["wrist_camera_serial"])
        self.base_camera_serial = str(dataset_camera_cfg["base_camera_serial"])
        self.base_roi_norm = tuple(
            float(value) for value in dataset_camera_cfg["base_roi_norm"]
        )
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.tab_4), "CAMERA1 腕部"
        )
        self._setup_d435_2_ui()

        # V2 owns the two physical RealSense RGB pipelines. The Qt window only
        # renders CameraSnapshot objects and keeps legacy frame caches for the
        # recorder compatibility bridge.
        self.wrist_camera_device = RealSenseRgbDevice(
            RealSenseRgbConfig(
                serial=self.wrist_camera_serial,
                width=640,
                height=480,
                fps=30,
            )
        )
        self.base_camera_device = RealSenseRgbDevice(
            RealSenseRgbConfig(
                serial=self.base_camera_serial,
                width=640,
                height=480,
                fps=30,
            )
        )
        self.D435_1_Started = False
        self.D435_2_Started = False

        self.btnD435_1_Start.clicked.connect(self.D435_1_Start)
        self.btnD435_1_Stop.clicked.connect(self.D435_1_Stop)
        self.btnD435_1_Cut.clicked.connect(self.D435_1_Cut)
        self.btnD435_2_Start.clicked.connect(self.D435_2_Start)
        self.btnD435_2_Stop.clicked.connect(self.D435_2_Stop)
        self.btnD435_2_Cut.clicked.connect(self.D435_2_Cut)


        # ROBOTIQ 力传感器部分
        self.btnForceSensor_INIT.clicked.connect(self.ForceSensor_1_INIT)

        # 设置定时器，每隔30ms刷新一次画面
        # LeRobot is the only active recorder in this copy. No HDF5 collector
        # or unrelated force-sensor polling thread is started.

    # RoArm-M2-Pro / CR5 / O6 teleoperation.................................
    def _setup_roarm_teleop(self):
        config_path = BASE_DIR / "config" / "roarm_cr5_teleop.json"
        self.teleop_store = TeleopConfigStore(config_path)
        cfg = self.teleop_store.data
        self.roarm_controller = RoArmSerialController(
            cfg["roarm"]["port"],
            cfg["roarm"]["baudrate"],
            cfg["roarm"]["feedback_hz"],
        )
        self.inverse3_controller = Inverse3Controller(
            cfg["inverse3"]["uri"],
            cfg["inverse3"]["device_id"],
            cfg["inverse3"]["grip_device_id"],
            cfg["inverse3"]["feedback_hz"],
            basis=cfg["inverse3"]["basis"],
        )
        gello_cfg = cfg["gello"]
        self.gello_controller = GelloController(
            port=gello_cfg["port"],
            software_root=gello_cfg["software_root"],
            joint_ids=gello_cfg["joint_ids"],
            joint_offsets=gello_cfg["joint_offsets"],
            joint_signs=gello_cfg["joint_signs"],
            gripper_config=gello_cfg["gripper_config"],
            baudrate=gello_cfg["baudrate"],
            feedback_hz=gello_cfg["feedback_hz"],
            feedback_timeout_s=gello_cfg["feedback_timeout_s"],
        )
        self.master_controller = (
            self.inverse3_controller
            if cfg["master"]["type"] == "inverse3"
            else self.gello_controller
            if cfg["master"]["type"] == "gello"
            else self.roarm_controller
        )
        self.o6_controller = O6Controller(
            cfg["o6"]["port"],
            BASE_DIR / "TESTHand_LINKERBOT" / "linker_hand_python_sdk" / "LinkerHand",
            cfg["o6"]["hand_id"],
            cfg["o6"]["baudrate"],
        )
        self.teleop_engine = TeleopEngine(
            self.teleop_store, self.master_controller, self.o6_controller
        )
        self._camera_frame_lock = threading.RLock()
        self._wrist_rgb_frame = None
        self._wrist_rgb_timestamp = 0.0
        self._base_rgb_frame = None
        self._base_roi_rgb_frame = None
        self._base_rgb_timestamp = 0.0
        self._workflow_prepare_active = False
        self._workflow_prepare_started_at = 0.0
        self._workflow_power_starting = False
        self._episode_operation_lock = threading.Lock()
        self._episode_operation_name = ""
        self._collection_accepted = False
        self._latest_robot_snapshot_at = 0.0
        self._last_robot_poll_started_at = 0.0
        self._latest_teleop_snapshot = None
        self._keyboard_shortcuts = {}
        self.shortcut_editors = {}
        log_dir = BASE_DIR / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / "gello_low_latency_test.log"
        self._file_logger = logging.Logger("gello.collection")
        log_handler = RotatingFileHandler(
            self.log_path, maxBytes=5 * 1024 * 1024, backupCount=3, encoding="utf-8"
        )
        log_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        self._file_logger.addHandler(log_handler)
        dataset_cfg = cfg["dataset"]
        self.lerobot_recorder = LeRobotEpisodeRecorder(
            sample_provider=self._lerobot_sample_provider,
            event_callback=self._teleop_event,
            worker_python=dataset_cfg["worker_python"],
            repo_prefix=dataset_cfg["repo_prefix"],
            fps=int(dataset_cfg["fps"]),
            image_size=tuple(dataset_cfg["image_size"]),
        )
        self._build_teleop_tab()
        self._build_gello_page()
        self.timer_teleop_ui = QTimer(self)
        self.timer_teleop_ui.timeout.connect(self.refresh_teleop_ui)
        self.timer_teleop_ui.start(100)
        self._teleop_event("info", f"ServoJ 实验副本启动；日志：{self.log_path}；未连接任何硬件")

    def _build_gello_page(self):
        self.gello_page = QScrollArea()
        self.gello_page.setWidgetResizable(True)
        self.gello_page.setFrameShape(QFrame.NoFrame)
        content = QWidget()
        self.gello_page.setWidget(content)
        root = QVBoxLayout(content)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(8)

        title = QLabel("GELLO  /  CR3A + O6 · ServoJ 跟随实验")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #174a7c;")
        root.addWidget(title)
        note = QLabel(
            "J1–J6 关节相对映射 · J7 控制 O6 · 启动跟随时建立当前姿态零点。\n"
            "独立测试副本：GELLO J1-J6 通过 NRC 7000 ServoJ 控制 CR3A，J7 控制 O6；"
            "可在参数维护中选择锁定 CR3A 关节；锁定关节保持跟随启动时姿态，不受 GELLO 控制；"
            "完整期望与已提交目标分开显示，耗时仅为软件观测。"
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #8a5300; background: #fff4df; padding: 10px;")
        root.addWidget(note)

        mapping_group = QGroupBox("GELLO 映射模式与比例")
        self.gello_mapping_group = mapping_group
        mapping_layout = QGridLayout(mapping_group)
        self.gello_page_control_mode = QComboBox()
        self.gello_page_control_mode.addItem(
            "J1-J6 → CR3A 关节；J7 → O6", "joint"
        )
        self.gello_page_control_mode.addItem(
            "J1-J5 → XYZ；J6 → CR3A J6（TCP IK）；J7 → O6", "xyz_j6_tcp_ik"
        )
        self.gello_page_control_mode.addItem(
            "J1-J5 → XYZ；J6 → CR3A J6（直接关节）", "xyz_j6"
        )
        self.gello_page_control_mode.addItem(
            "J1-J6 → CR3A 完整 TCP 6D IK；J7 → O6（实验模式）", "tcp_6d"
        )
        self.gello_page_control_mode.setCurrentIndex(
            self.gello_page_control_mode.findData(
                self.teleop_store.data["gello"].get("control_mode", "joint")
            )
        )
        mapping_layout.addWidget(QLabel("当前模式"), 0, 0)
        mapping_layout.addWidget(self.gello_page_control_mode, 0, 1, 1, 3)
        mapping_layout.addWidget(
            QLabel("已验证路径使用 GELLO 关节相对增量；下面的 XYZ/TCP 选项仅用于后续实验。"),
            1,
            0,
            1,
            4,
        )
        mapping_layout.addWidget(QLabel("XYZ 比例"), 2, 0)
        self.gello_page_xyz_scale_spins = []
        for index, axis in enumerate(("X", "Y", "Z")):
            mapping_layout.addWidget(QLabel(axis), 2, index + 1)
            spin = self._make_double_spin(
                self.teleop_store.data["gello"]["xyz_scale"][index],
                -10.0,
                10.0,
                0.1,
                2,
                " ×",
            )
            self.gello_page_xyz_scale_spins.append(spin)
            mapping_layout.addWidget(spin, 3, index + 1)
        self.gello_page_mapping_apply = QPushButton("保存并应用映射比例")
        self.gello_page_mapping_apply.clicked.connect(self.save_teleop_settings)
        mapping_layout.addWidget(self.gello_page_mapping_apply, 4, 0, 1, 4)
        mapping_layout.addWidget(QLabel("坐标轴对齐"), 5, 0)
        axis_options = (
            ("+ GELLO FK X", [1.0, 0.0, 0.0]),
            ("- GELLO FK X", [-1.0, 0.0, 0.0]),
            ("+ GELLO FK Y", [0.0, 1.0, 0.0]),
            ("- GELLO FK Y", [0.0, -1.0, 0.0]),
            ("+ GELLO FK Z", [0.0, 0.0, 1.0]),
            ("- GELLO FK Z", [0.0, 0.0, -1.0]),
        )
        configured_axis_map = self.teleop_store.data["gello"]["xyz_axis_map"]
        self.gello_page_axis_map_combos = []
        for row, axis in enumerate(("CR3A X", "CR3A Y", "CR3A Z"), start=6):
            combo = QComboBox()
            for label, vector in axis_options:
                combo.addItem(label, vector)
            current = [float(value) for value in configured_axis_map[row - 6]]
            current_index = next(
                index
                for index, (_label, vector) in enumerate(axis_options)
                if vector == current
            )
            combo.setCurrentIndex(current_index)
            self.gello_page_axis_map_combos.append(combo)
            mapping_layout.addWidget(QLabel(axis), row, 0)
            mapping_layout.addWidget(combo, row, 1, 1, 3)
        mapping_layout.addWidget(
            QLabel("每个 CR3A 轴选择一个 GELLO FK 轴及正负方向；先保持从臂停止，再保存测试。"),
            9,
            0,
            1,
            4,
        )
        # Experimental mapping controls retain their configuration, but are
        # deliberately absent from the production collection navigation.
        mapping_group.setParent(self.teleop_content)
        mapping_group.hide()

        status_group = QGroupBox("状态")
        status_layout = QVBoxLayout(status_group)
        status_layout.setContentsMargins(8, 8, 8, 8)
        status_layout.setSpacing(4)
        self.gello_page_state_label = QLabel("空闲")
        self.gello_page_device_label = QLabel("CR3A：未连接 | GELLO：未连接 | O6：未连接")
        self.gello_page_joint_label = QLabel("GELLO 关节：--")
        self.gello_page_camera_label = QLabel("相机：请打开采集操作台查看三路画面")
        self.gello_page_lerobot_label = QLabel("LeRobot：请打开采集操作台开始记录")
        self.gello_stream_label = QLabel("指令流：未启动 | 实际反馈：--")
        self.gello_latency_label = QLabel("ServoJ 软件时序：尚未测量")
        for label in (
            self.gello_page_state_label,
            self.gello_page_device_label,
            self.gello_page_joint_label,
            self.gello_stream_label,
            self.gello_latency_label,
        ):
            label.setWordWrap(True)
            status_layout.addWidget(label)
        for label in (self.gello_page_camera_label, self.gello_page_lerobot_label):
            label.setParent(status_group)
            label.hide()
        self.gello_joint_table = QTableWidget(6, 7)
        self.gello_joint_table.setHorizontalHeaderLabels(
            [
                "关节", "GELLO / °", "CR3A 实际 / °", "完整期望 / °",
                "最近有效 / °", "实际提交 / °", "期望−实际 / °",
            ]
        )
        self.gello_joint_table.verticalHeader().hide()
        self.gello_joint_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.gello_joint_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.gello_joint_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.gello_joint_table.setAlternatingRowColors(True)
        self.gello_joint_table.verticalHeader().setDefaultSectionSize(24)
        self.gello_joint_table.setFixedHeight(176)
        for row in range(6):
            for column in range(7):
                item = QTableWidgetItem(f"J{row + 1}" if column == 0 else "--")
                item.setTextAlignment(Qt.AlignCenter)
                self.gello_joint_table.setItem(row, column, item)
        status_layout.addWidget(self.gello_joint_table)
        root.addWidget(status_group)

        control_group = QGroupBox("操作顺序")
        control_layout = QGridLayout(control_group)
        self.gello_page_connect_robot = QPushButton("1. 连接 CR3A")
        self.gello_page_power_on = QPushButton("3. CR3A 上使能")
        self.gello_page_clear_error = QPushButton("清错并上使能")
        self.gello_page_connect_devices = QPushButton("2. 连接 GELLO + O6")
        self.gello_page_start = QPushButton("4. 开始 GELLO 跟随")
        self.gello_page_stop = QPushButton("停止并保持从臂")
        self.gello_page_estop = QPushButton("软件紧急停止")
        self.gello_page_estop.setStyleSheet(
            "background-color: #c62828; color: white; font-weight: bold;"
        )
        self.gello_page_connect_robot.clicked.connect(self.RobotCONNECT)
        self.gello_page_power_on.clicked.connect(self.WorkflowPowerOn)
        self.gello_page_clear_error.clicked.connect(self.RobotClearError)
        self.gello_page_connect_devices.clicked.connect(self.RoArmDevicesConnect)
        self.gello_page_start.clicked.connect(self.WorkflowStartTeleop)
        self.gello_page_stop.clicked.connect(self.TeleopStopCurrentAction)
        self.gello_page_estop.clicked.connect(self.EmergencyStop)
        control_layout.addWidget(self.gello_page_connect_robot, 0, 0)
        control_layout.addWidget(self.gello_page_connect_devices, 0, 1)
        control_layout.addWidget(self.gello_page_power_on, 0, 2)
        control_layout.addWidget(self.gello_page_start, 0, 3)
        self.gello_read_only_button = QPushButton("仅连接 GELLO（只读检查）")
        self.gello_read_only_button.clicked.connect(self.MasterConnect)
        control_layout.addWidget(self.gello_read_only_button, 1, 0, 1, 2)
        control_layout.addWidget(QLabel("先检查现场与实际反馈，再启用跟随；不会自动上使能。"), 1, 2, 1, 2)
        root.insertWidget(2, control_group)

        limit_group = QGroupBox("GELLO 关节安全限位")
        self.gello_limits_group = limit_group
        limit_layout = QGridLayout(limit_group)
        self.gello_page_step_limit = self._make_double_spin(
            math.degrees(float(self.teleop_store.data["robot"]["safety_max_command_step_rad"])),
            1.0,
            360.0,
            1.0,
            1,
            " °",
        )
        self.gello_page_tracking_limit = self._make_double_spin(
            math.degrees(float(self.teleop_store.data["robot"]["safety_max_tracking_error_rad"])),
            1.0,
            360.0,
            1.0,
            1,
            " °",
        )
        self.gello_page_speed_limit = self._make_double_spin(
            float(self.teleop_store.data["robot"]["safety_max_command_speed_rad_s"]),
            0.1,
            20.0,
            0.1,
            2,
            " rad/s",
        )
        self.gello_page_limit_apply = QPushButton("保存并应用限位")
        self.gello_page_limit_apply.clicked.connect(self._apply_gello_page_limits)
        limit_layout.addWidget(QLabel("单帧跳变上限"), 0, 0)
        limit_layout.addWidget(self.gello_page_step_limit, 0, 1)
        limit_layout.addWidget(QLabel("跟踪误差上限"), 0, 2)
        limit_layout.addWidget(self.gello_page_tracking_limit, 0, 3)
        limit_layout.addWidget(QLabel("速度保护上限"), 1, 0)
        limit_layout.addWidget(self.gello_page_speed_limit, 1, 1)
        limit_layout.addWidget(self.gello_page_limit_apply, 2, 0, 1, 2)
        limit_layout.addWidget(
            QLabel("超过速度上限会自动停止；建议先从 0.8 rad/s 逐步增加"),
            2,
            2,
            1,
            2,
        )

        home_group = QGroupBox("HOME / 采集")
        home_layout = QGridLayout(home_group)
        self.gello_page_save_home = QPushButton("保存当前为 HOME")
        self.gello_page_go_home = QPushButton("CR3A / O6 回 HOME")
        self.gello_page_save_home.setToolTip("点击后立即用当前姿态覆盖 HOME；必须先停止跟随并处理 Episode")
        self.gello_page_go_home.setToolTip("点击后立即执行 HOME 回位；请先确认路径、工作区和硬件急停")
        self.gello_page_collection = QPushButton("进入数采工作台 →")
        self.gello_page_save_home.clicked.connect(self.TeleopSaveHome)
        self.gello_page_go_home.clicked.connect(self.TeleopGoHome)
        self.gello_page_collection.clicked.connect(
            lambda: self.tabWidget.setCurrentWidget(self.tab_4)
        )
        home_layout.addWidget(self.gello_page_save_home, 0, 0)
        home_layout.addWidget(self.gello_page_go_home, 0, 1)
        home_layout.addWidget(self.gello_page_collection, 0, 2, 1, 2)
        root.addWidget(home_group)
        root.addStretch(1)
        self.tabWidget.addTab(self.gello_page, "GELLO 遥操作")

    def _apply_gello_page_limits(self):
        self.teleop_joint_step_limit.setValue(self.gello_page_step_limit.value())
        self.teleop_tracking_limit.setValue(self.gello_page_tracking_limit.value())
        self.save_teleop_settings()

    @staticmethod
    def _make_double_spin(value, minimum, maximum, step, decimals=2, suffix=""):
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(float(value))
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def _build_teleop_tab(self):
        cfg = self.teleop_store.data
        self.teleop_tab = QScrollArea()
        self.teleop_tab.setWidgetResizable(True)
        self.teleop_tab.setFrameShape(QFrame.NoFrame)
        self.teleop_content = QWidget()
        self.teleop_tab.setWidget(self.teleop_content)
        root = QVBoxLayout(self.teleop_content)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        status_group = QGroupBox("连接与状态")
        self.teleop_status_group = status_group
        status_layout = QGridLayout(status_group)
        self.teleop_robot_ip = QLineEdit(str(cfg["robot"]["ip"]))
        self.teleop_master_type = QComboBox()
        self.teleop_master_type.addItem("Inverse3 + VerseGrip（XYZ + RPY）", "inverse3")
        self.teleop_master_type.addItem("RoArm-M2-Pro（XYZ，锁定 RPY）", "roarm")
        self.teleop_master_type.addItem(
            "GELLO（J1-J6 关节相对跟随，J7→O6）", "gello"
        )
        self.teleop_master_type.setCurrentIndex(
            self.teleop_master_type.findData(cfg["master"]["type"])
        )
        self.teleop_roarm_port = QLineEdit(str(cfg["roarm"]["port"]))
        self.teleop_inverse3_uri = QLineEdit(str(cfg["inverse3"]["uri"]))
        self.teleop_gello_port = QLineEdit(str(cfg["gello"]["port"]))
        self.teleop_o6_port = QLineEdit(str(cfg["o6"]["port"]))
        self.teleop_roarm_port.setCursorPosition(0)
        self.teleop_inverse3_uri.setCursorPosition(0)
        self.teleop_gello_port.setCursorPosition(0)
        self.teleop_o6_port.setCursorPosition(0)
        self.teleop_state_label = QLabel("空闲")
        self.teleop_state_label.setStyleSheet("font-weight: bold; color: #555;")
        self.teleop_device_label = QLabel("CR5: 未连接 | 主臂: 未连接 | O6: 未连接")
        self.teleop_master_label = QLabel("主臂: --")
        self.teleop_hand_label = QLabel("O6: --")
        status_layout.addWidget(QLabel("控制柜 IP"), 0, 0)
        status_layout.addWidget(self.teleop_robot_ip, 0, 1)
        status_layout.addWidget(QLabel("主臂类型"), 1, 0)
        status_layout.addWidget(self.teleop_master_type, 1, 1)
        status_layout.addWidget(QLabel("RoArm 串口"), 2, 0)
        status_layout.addWidget(self.teleop_roarm_port, 2, 1)
        status_layout.addWidget(QLabel("Inverse Service"), 3, 0)
        status_layout.addWidget(self.teleop_inverse3_uri, 3, 1)
        status_layout.addWidget(QLabel("GELLO 串口"), 4, 0)
        status_layout.addWidget(self.teleop_gello_port, 4, 1)
        status_layout.addWidget(QLabel("O6 串口"), 5, 0)
        status_layout.addWidget(self.teleop_o6_port, 5, 1)
        status_layout.addWidget(QLabel("运行状态"), 0, 2)
        status_layout.addWidget(self.teleop_state_label, 0, 3)
        status_layout.addWidget(self.teleop_device_label, 1, 2, 1, 2)
        status_layout.addWidget(self.teleop_master_label, 2, 2, 1, 2)
        status_layout.addWidget(self.teleop_hand_label, 3, 2, 1, 2)
        status_layout.setColumnStretch(1, 3)
        status_layout.setColumnStretch(3, 2)
        root.addWidget(status_group)

        parameters = QGroupBox("XYZ / RPY 映射与安全限制")
        self.teleop_mapping_group = parameters
        parameter_layout = QGridLayout(parameters)
        axes = ("X", "Y", "Z")
        self.teleop_scale_spins = []
        self.teleop_limit_spins = []
        for index, axis in enumerate(axes):
            parameter_layout.addWidget(QLabel(axis), 0, index + 1)
            scale = self._make_double_spin(
                cfg["teleop"]["scale_xyz"][index], -10.0, 10.0, 0.1, 2, " ×"
            )
            limit = self._make_double_spin(
                cfg["teleop"]["max_delta_xyz_mm"][index], 1.0, 500.0, 5.0, 1, " mm"
            )
            self.teleop_scale_spins.append(scale)
            self.teleop_limit_spins.append(limit)
            parameter_layout.addWidget(scale, 1, index + 1)
            parameter_layout.addWidget(limit, 2, index + 1)
        parameter_layout.addWidget(QLabel("位移倍率"), 1, 0)
        parameter_layout.addWidget(QLabel("相对范围 ±"), 2, 0)
        rpy_axes = ("A/Roll", "B/Pitch", "C/Yaw")
        self.teleop_rpy_scale_spins = []
        self.teleop_rpy_limit_spins = []
        for index, axis in enumerate(rpy_axes):
            parameter_layout.addWidget(QLabel(axis), 3, index + 1)
            scale = self._make_double_spin(
                cfg["teleop"]["scale_rpy"][index], -10.0, 10.0, 0.1, 2, " ×"
            )
            limit = self._make_double_spin(
                cfg["teleop"]["max_delta_rpy_rad"][index],
                0.01,
                3.14,
                0.05,
                3,
                " rad",
            )
            self.teleop_rpy_scale_spins.append(scale)
            self.teleop_rpy_limit_spins.append(limit)
            parameter_layout.addWidget(scale, 4, index + 1)
            parameter_layout.addWidget(limit, 5, index + 1)
        parameter_layout.addWidget(QLabel("姿态倍率"), 4, 0)
        parameter_layout.addWidget(QLabel("姿态范围 ±"), 5, 0)
        self.teleop_joint_step_limit = self._make_double_spin(
            math.degrees(float(cfg["robot"]["safety_max_command_step_rad"])),
            1.0,
            360.0,
            1.0,
            1,
            " °",
        )
        self.teleop_tracking_limit = self._make_double_spin(
            math.degrees(float(cfg["robot"]["safety_max_tracking_error_rad"])),
            1.0,
            360.0,
            1.0,
            1,
            " °",
        )
        self.teleop_joint_step_limit.setToolTip(
            "GELLO 单次有效采样允许的最大关节目标变化；超过后安全停止"
        )
        self.teleop_tracking_limit.setToolTip(
            "CR3A 实际关节落后目标的最大误差；超过后安全停止"
        )
        parameter_layout.addWidget(QLabel("GELLO 单帧关节跳变 ±"), 7, 0)
        parameter_layout.addWidget(self.teleop_joint_step_limit, 7, 1)
        parameter_layout.addWidget(QLabel("从臂跟踪误差 ±"), 7, 2)
        parameter_layout.addWidget(self.teleop_tracking_limit, 7, 3)
        self.teleop_mapping_apply_button = QPushButton("保存并立即应用 XYZ / RPY 映射")
        self.teleop_mapping_apply_button.clicked.connect(self.save_teleop_settings)
        self.teleop_mapping_status_label = QLabel(
            "Inverse3 使用 VerseGrip 相对四元数控制 RPY；倍率变化时重建主从零点"
        )
        self.teleop_mapping_status_label.setWordWrap(True)
        self.teleop_mapping_status_label.setStyleSheet("color: #25627a;")
        parameter_layout.addWidget(self.teleop_mapping_apply_button, 8, 0, 1, 2)
        parameter_layout.addWidget(self.teleop_mapping_status_label, 8, 2, 1, 2)
        root.addWidget(parameters)

        speed_group = QGroupBox("运动速度（CR5 / RoArm / O6）")
        self.teleop_speed_group = speed_group
        speed_layout = QGridLayout(speed_group)
        self.teleop_tcp_speed = self._make_double_spin(
            cfg["teleop"]["max_tcp_speed_mm_s"], 1.0, 200.0, 1.0, 1, " mm/s"
        )
        self.teleop_tcp_speed.setToolTip(
            "XYZ 跟随时 CR5 TCP 的最大平移速度；修改后可在正在运行的跟随中即时生效"
        )
        self.teleop_rpy_speed = self._make_double_spin(
            cfg["teleop"]["max_tcp_angular_speed_rad_s"],
            0.01,
            2.0,
            0.01,
            2,
            " rad/s",
        )
        self.teleop_preset_speed = self._make_double_spin(
            cfg["preset"]["robot_velocity_percent"], 1.0, 100.0, 1.0, 1, " %"
        )
        self.teleop_preset_speed.setToolTip(
            "CR5 回 HOME、A/B/C/D 示教点以及旧页面 MoveJ/MoveL 使用的速度"
        )
        self.teleop_preset_acc = self._make_double_spin(
            cfg["preset"]["robot_acc_percent"], 1.0, 100.0, 1.0, 1, " %"
        )
        self.teleop_preset_dec = self._make_double_spin(
            cfg["preset"]["robot_dec_percent"], 1.0, 100.0, 1.0, 1, " %"
        )
        self.teleop_roarm_duration = self._make_double_spin(
            cfg["preset"]["roarm_duration_s"], 1.0, 15.0, 0.5, 1, " s"
        )
        self.teleop_roarm_duration.setToolTip(
            "RoArm 主臂主动回 HOME/示教点所需时长；数值越大，运动越慢"
        )
        configured_o6_speeds = [int(value) for value in cfg["o6"]["speed"]]
        self.teleop_o6_speed = QSpinBox()
        self.teleop_o6_speed.setRange(1, 255)
        self.teleop_o6_speed.setSingleStep(5)
        self.teleop_o6_speed.setValue(int(round(sum(configured_o6_speeds) / 6.0)))
        self.teleop_o6_speed.setSuffix(" /255")
        self.teleop_o6_speed.setToolTip(
            "统一设置 O6 六个电机的速度；只改速度，不改变力矩和当前位置"
        )
        self.teleop_replay_initial_speed = self._make_double_spin(
            cfg["replay"]["initial_move_velocity_percent"],
            1.0,
            100.0,
            1.0,
            1,
            " %",
        )
        self.teleop_replay_initial_speed.setToolTip(
            "仅在配置关闭相对重放时使用：CR5 移动到录制首帧的 MoveJ 速度；"
            "当前相对重放不会先跳回旧录制姿态"
        )
        speed_layout.addWidget(QLabel("XYZ 跟随最高速度"), 0, 0)
        speed_layout.addWidget(self.teleop_tcp_speed, 0, 1)
        speed_layout.addWidget(QLabel("CR5 回位 MoveJ 速度"), 0, 2)
        speed_layout.addWidget(self.teleop_preset_speed, 0, 3)
        speed_layout.addWidget(QLabel("CR5 加速度"), 1, 0)
        speed_layout.addWidget(self.teleop_preset_acc, 1, 1)
        speed_layout.addWidget(QLabel("CR5 减速度"), 1, 2)
        speed_layout.addWidget(self.teleop_preset_dec, 1, 3)
        speed_layout.addWidget(QLabel("RoArm 主动回位时长"), 2, 0)
        speed_layout.addWidget(self.teleop_roarm_duration, 2, 1)
        speed_layout.addWidget(QLabel("O6 六电机统一速度"), 2, 2)
        speed_layout.addWidget(self.teleop_o6_speed, 2, 3)
        speed_layout.addWidget(QLabel("绝对 Replay 首帧速度"), 3, 0)
        speed_layout.addWidget(self.teleop_replay_initial_speed, 3, 1)
        speed_layout.addWidget(QLabel("RPY 跟随最高速度"), 4, 0)
        speed_layout.addWidget(self.teleop_rpy_speed, 4, 1)
        self.teleop_apply_speed_button = QPushButton("保存并立即应用速度")
        self.teleop_apply_speed_button.clicked.connect(self.save_teleop_settings)
        self.teleop_speed_status_label = QLabel(
            "XYZ/O6 可即时应用；CR5 回位和 Replay 速度从下一次动作开始生效"
        )
        self.teleop_speed_status_label.setWordWrap(True)
        self.teleop_speed_status_label.setStyleSheet("color: #25627a;")
        speed_layout.addWidget(self.teleop_apply_speed_button, 3, 2)
        speed_layout.addWidget(self.teleop_speed_status_label, 3, 3)
        for column in range(4):
            speed_layout.setColumnStretch(column, 1)
        root.addWidget(speed_group)

        action_row = QHBoxLayout()
        self.teleop_connect_robot_button = QPushButton("连接纳博特 CR5")
        self.teleop_connect_master_button = QPushButton("只读连接主臂")
        self.teleop_connect_devices_button = QPushButton("连接主臂 + O6")
        self.teleop_save_settings_button = QPushButton("保存并应用参数")
        self.teleop_power_on_button = QPushButton("CR5 上使能（不跟随）")
        self.teleop_start_button = QPushButton("开始主从跟随")
        self.teleop_stop_button = QPushButton("停止当前动作并保持从臂")
        self.teleop_estop_button = QPushButton("软件紧急停止")
        self.teleop_estop_button.setStyleSheet(
            "background-color: #c62828; color: white; font-weight: bold;"
        )
        self.teleop_connect_robot_button.clicked.connect(self.RobotCONNECT)
        self.teleop_connect_master_button.clicked.connect(self.MasterConnect)
        self.teleop_connect_devices_button.clicked.connect(self.RoArmDevicesConnect)
        self.teleop_save_settings_button.clicked.connect(self.save_teleop_settings)
        self.teleop_power_on_button.clicked.connect(self.WorkflowPowerOn)
        self.teleop_start_button.clicked.connect(self.TeleopFollowStart)
        self.teleop_stop_button.clicked.connect(self.TeleopStopCurrentAction)
        self.teleop_estop_button.clicked.connect(self.EmergencyStop)
        for button in (
            self.teleop_connect_robot_button,
            self.teleop_connect_master_button,
            self.teleop_connect_devices_button,
            self.teleop_save_settings_button,
            self.teleop_power_on_button,
            self.teleop_start_button,
            self.teleop_stop_button,
            self.teleop_estop_button,
        ):
            action_row.addWidget(button)
        root.addLayout(action_row)

        mode_row = QHBoxLayout()
        self.teleop_master_free_button = QPushButton("从臂保持 / 主臂自由")
        self.teleop_save_home_button = QPushButton("保存当前为初始位")
        self.teleop_home_button = QPushButton("主从臂一键回初始位")
        self.teleop_master_free_button.setToolTip(
            "CR5 和 O6 保持当前位置，RoArm 全部卸力，可徒手拖动"
        )
        self.teleop_save_home_button.setToolTip(
            "同时保存 RoArm、CR5 和 O6 当前位置为独立 HOME 初始位"
        )
        self.teleop_home_button.setToolTip(
            "CR5 使用 MoveJ，RoArm 和 O6 同时回到已保存 HOME，到位后保持"
        )
        self.teleop_master_free_button.clicked.connect(self.TeleopMasterFree)
        self.teleop_save_home_button.clicked.connect(self.TeleopSaveHome)
        self.teleop_home_button.clicked.connect(self.TeleopGoHome)
        mode_row.addWidget(self.teleop_master_free_button)
        mode_row.addWidget(self.teleop_save_home_button)
        mode_row.addWidget(self.teleop_home_button)
        root.addLayout(mode_row)

        recovery_group = QGroupBox("O6 电机故障恢复")
        self.teleop_recovery_group = recovery_group
        recovery_layout = QHBoxLayout(recovery_group)
        recovery_layout.addWidget(QLabel("电机"))
        self.teleop_o6_motor_combo = QComboBox()
        for number, motor_name in enumerate(O6_MOTOR_NAMES, start=1):
            self.teleop_o6_motor_combo.addItem(f"{number} - {motor_name}", number)
        self.teleop_o6_motor_combo.setCurrentIndex(2)
        self.teleop_o6_recover_button = QPushButton("恢复所选 O6 电机（安全重发）")
        self.teleop_o6_recover_button.setToolTip(
            "停止跟随后，先将所选电机目标设为当前反馈位置，再重发速度和力矩；"
            "O6 SDK 没有独立的使能/清故障命令"
        )
        self.teleop_o6_recover_button.clicked.connect(self.TeleopRecoverO6Motor)
        recovery_note = QLabel("默认 3 号=食指；过温/编码器/电压故障会拒绝上力")
        recovery_note.setStyleSheet("color: #9a5b00;")
        recovery_layout.addWidget(self.teleop_o6_motor_combo)
        recovery_layout.addWidget(self.teleop_o6_recover_button)
        recovery_layout.addWidget(recovery_note, 1)
        root.addWidget(recovery_group)

        o6_action_group = QGroupBox("O6 快捷动作")
        self.teleop_o6_action_group = o6_action_group
        o6_action_layout = QGridLayout(o6_action_group)
        o6_action_note = QLabel(
            "跟随时 GELLO J7 控制 O6：J7 张开触发“张开手”，J7 闭合触发配置中的“抓取”；"
            "下方按钮仅用于停止跟随后的手爪检查。"
        )
        o6_action_note.setStyleSheet("color: #9a5b00;")
        o6_action_layout.addWidget(o6_action_note, 0, 0, 1, 4)
        self.teleop_o6_action_buttons = {}
        actions = cfg["o6"]["actions"]
        for index, (name, target) in enumerate(actions.items()):
            button = QPushButton(name)
            button.setToolTip(f"O6 目标：{list(target)}")
            button.clicked.connect(
                lambda _checked=False, key=name: self.TeleopO6Action(key)
            )
            self.teleop_o6_action_buttons[name] = button
            o6_action_layout.addWidget(button, 1 + index // 4, index % 4)
        action_rows = (len(actions) + 3) // 4
        mode_row = 1 + action_rows
        self.teleop_o6_mode_label = QLabel("O6 控制：M5 跟随")
        self.teleop_o6_m5_button = QPushButton("恢复 M5 控制 O6")
        self.teleop_o6_m5_button.setToolTip(
            "退出灵巧手独立动作模式；XYZ 跟随时，RoArm M5 将再次控制 O6 开合"
        )
        self.teleop_o6_m5_button.clicked.connect(self.TeleopEnableM5Control)
        o6_action_layout.addWidget(self.teleop_o6_mode_label, mode_row, 0, 1, 2)
        o6_action_layout.addWidget(self.teleop_o6_m5_button, mode_row, 2, 1, 2)
        root.addWidget(o6_action_group)

        replay_group = QGroupBox("CR5 可选关节 + O6 同步轨迹录制 / Replay")
        self.teleop_replay_group = replay_group
        replay_layout = QGridLayout(replay_group)
        replay_note = QLabel(
            "动作记录 / 重放：只重放勾选的 CR5 关节，未选关节保持重放开始位置；"
            "O6 同步重放。点击或按快捷键后立即执行，不再弹出二次确认；"
            "Episode 录制中重放会连续采集，完成后自动恢复 XYZ 跟随"
        )
        replay_note.setStyleSheet("color: #9a5b00;")
        replay_joint_row = QHBoxLayout()
        replay_joint_row.addWidget(QLabel("CR5 参与关节："))
        self.teleop_replay_joint_checks = {}
        configured_joint_indices = set(cfg["replay"]["robot_joint_indices"])
        for joint_number in range(1, 7):
            checkbox = QCheckBox(f"J{joint_number}")
            checkbox.setChecked(joint_number in configured_joint_indices)
            checkbox.setToolTip(
                "勾选：重放该关节的录制轨迹；未勾选：保持重放开始时的位置"
            )
            self.teleop_replay_joint_checks[joint_number] = checkbox
            replay_joint_row.addWidget(checkbox)
        self.teleop_replay_select_all_button = QPushButton("全选")
        self.teleop_replay_only_j6_button = QPushButton("仅 J6")
        self.teleop_replay_select_all_button.clicked.connect(
            lambda _checked=False: self._set_replay_joint_selection(range(1, 7))
        )
        self.teleop_replay_only_j6_button.clicked.connect(
            lambda _checked=False: self._set_replay_joint_selection((6,))
        )
        replay_joint_row.addWidget(self.teleop_replay_select_all_button)
        replay_joint_row.addWidget(self.teleop_replay_only_j6_button)
        replay_joint_row.addStretch(1)
        self.teleop_record_start_buttons = {}
        self.teleop_record_stop_buttons = {}
        self.teleop_replay_start_buttons = {}
        self.teleop_replay_slot_status_labels = {}
        for slot in (1, 2, 3):
            record_start = QPushButton(f"录制{slot}")
            record_stop = QPushButton(f"停止并保存{slot}")
            replay_start = QPushButton(f"回放{slot}")
            status_label = QLabel(f"轨迹{slot}：尚未录制")
            record_start.clicked.connect(
                lambda _checked=False, slot=slot: self.TeleopRecordStart(slot)
            )
            record_stop.clicked.connect(
                lambda _checked=False, slot=slot: self.TeleopRecordStop(slot)
            )
            replay_start.clicked.connect(
                lambda _checked=False, slot=slot: self.TeleopReplayStart(slot)
            )
            self.teleop_record_start_buttons[slot] = record_start
            self.teleop_record_stop_buttons[slot] = record_stop
            self.teleop_replay_start_buttons[slot] = replay_start
            self.teleop_replay_slot_status_labels[slot] = status_label
            row = slot + 1
            replay_layout.addWidget(QLabel(f"轨迹 {slot}"), row, 0)
            replay_layout.addWidget(record_start, row, 1)
            replay_layout.addWidget(record_stop, row, 2)
            replay_layout.addWidget(replay_start, row, 3)
            replay_layout.addWidget(status_label, row, 4)

        # Keep the original attributes as slot-1 aliases for the R shortcut and
        # external code that already addresses the former single-slot controls.
        self.teleop_record_start_button = self.teleop_record_start_buttons[1]
        self.teleop_record_stop_button = self.teleop_record_stop_buttons[1]
        self.teleop_replay_start_button = self.teleop_replay_start_buttons[1]
        self.teleop_replay_stop_button = QPushButton("停止重放")
        self.teleop_replay_status_label = QLabel("三段轨迹独立保存；快捷键 R 回放轨迹 1")
        self.teleop_replay_stop_button.clicked.connect(self.TeleopReplayStop)
        replay_layout.addWidget(replay_note, 0, 0, 1, 5)
        replay_layout.addLayout(replay_joint_row, 1, 0, 1, 5)
        replay_layout.addWidget(self.teleop_replay_status_label, 5, 0, 1, 4)
        replay_layout.addWidget(self.teleop_replay_stop_button, 5, 4)
        replay_layout.setColumnStretch(4, 2)
        root.addWidget(replay_group)

        dataset_group = QGroupBox("LeRobot / PI0.5 Episode 采集")
        self.teleop_dataset_group = dataset_group
        dataset_layout = QGridLayout(dataset_group)
        dataset_cfg = cfg["dataset"]
        dataset_layout.addWidget(QLabel("任务文字"), 0, 0)
        self.lerobot_task_edit = QLineEdit(str(dataset_cfg["task"]))
        self.lerobot_task_edit.setToolTip(
            "PI0.5 的语言指令；同一任务建议始终使用同一句英文"
        )
        dataset_layout.addWidget(self.lerobot_task_edit, 0, 1, 1, 3)
        dataset_layout.addWidget(QLabel("数据集根目录"), 1, 0)
        self.lerobot_root_edit = QLineEdit(str(dataset_cfg["root"]))
        self.lerobot_root_edit.setToolTip(
            "每次启动会在此目录下创建带时间戳的新数据集"
        )
        dataset_layout.addWidget(self.lerobot_root_edit, 1, 1, 1, 3)
        self.lerobot_start_button = QPushButton("开始 Episode")
        self.lerobot_save_button = QPushButton("成功并保存")
        self.lerobot_discard_button = QPushButton("失败并丢弃")
        self.lerobot_finalize_button = QPushButton("结束数据集")
        self.lerobot_start_button.clicked.connect(self.LeRobotEpisodeStart)
        self.lerobot_save_button.clicked.connect(self.LeRobotEpisodeSave)
        self.lerobot_discard_button.clicked.connect(self.LeRobotEpisodeDiscard)
        self.lerobot_finalize_button.clicked.connect(self.LeRobotDatasetFinalize)
        dataset_layout.addWidget(self.lerobot_start_button, 2, 0)
        dataset_layout.addWidget(self.lerobot_save_button, 2, 1)
        dataset_layout.addWidget(self.lerobot_discard_button, 2, 2)
        dataset_layout.addWidget(self.lerobot_finalize_button, 2, 3)
        self.lerobot_status_label = QLabel("尚未启动 LeRobot 数据集")
        self.lerobot_status_label.setWordWrap(True)
        dataset_layout.addWidget(self.lerobot_status_label, 3, 0, 1, 4)
        dataset_note = QLabel(
            "录制三路 RGB：CAMERA2 基座全图、CAMERA1 腕部全图、CAMERA2 ROI；"
            f"每路先按 OpenPI 规则等比例缩放+居中黑边到 "
            f"{int(dataset_cfg['image_size'][1])}×{int(dataset_cfg['image_size'][0])}，"
            "再用 H.264 CRF 28 写入 MP4；不保存 640×480 原图和深度图。"
            "State 为 CR5 六关节 + TCP XYZ/RPY + O6 六路反馈，共 18 维；"
            "Action 为 CR5 ΔTCP（m/rad）+ O6 六路目标，共 12 维。"
            "Episode 内插入轨迹也会连续记录正确的腕部旋转增量。\n"
            "连续采集流程：开始 Episode → 结束录制 → 选择结果并保存当前 → 再次开始 Episode；"
            "同一数据集会话中的 Episode 会分别保存，不会覆盖前一条。"
        )
        dataset_note.setStyleSheet("color: #25627a;")
        dataset_note.setWordWrap(True)
        dataset_layout.addWidget(dataset_note, 4, 0, 1, 4)
        root.addWidget(dataset_group)

        preset_group = QGroupBox("同步示教点（A/B/C/D）")
        self.teleop_preset_group = preset_group
        preset_layout = QGridLayout(preset_group)
        self.teleop_preset_note = QLabel(
            "默认键盘 A/B/C/D：回位；Shift+A/B/C/D：保存（可在采集操作台修改）"
        )
        preset_layout.addWidget(self.teleop_preset_note, 0, 0, 1, 4)
        self.teleop_recall_buttons = {}
        self.teleop_save_buttons = {}
        for index, name in enumerate("ABCD"):
            recall = QPushButton(f"回到 {name}")
            save = QPushButton(f"保存 {name}")
            recall.clicked.connect(lambda _checked=False, key=name: self.TeleopRecallPreset(key))
            save.clicked.connect(lambda _checked=False, key=name: self.TeleopSavePreset(key))
            self.teleop_recall_buttons[name] = recall
            self.teleop_save_buttons[name] = save
            preset_layout.addWidget(recall, 1, index)
            preset_layout.addWidget(save, 2, index)
        root.addWidget(preset_group)

        note = QLabel(
            "跟随启动时记录主从零点：RoArm 模式只映射 XYZ 并锁定 RPY；"
            "Inverse3 模式使用 VerseGrip 四元数映射相对 RPY，缺少 VerseGrip 时拒绝启动。"
            "软件停止不能替代控制柜硬件急停。首次实机跟随建议先把倍率设为 0.5。"
        )
        note.setWordWrap(True)
        root.addWidget(note)
        self.teleop_log = QPlainTextEdit()
        self.teleop_log.setReadOnly(True)
        self.teleop_log.setMaximumHeight(110)
        root.addWidget(self.teleop_log)
        self.tabWidget.addTab(self.teleop_tab, "高级遥操设置")

    def _setup_responsive_main_layout(self):
        """Keep safety controls and the bounded event log visible on every tab."""
        self.setMinimumSize(1120, 760)
        self.resize(1360, 940)
        main_layout = QVBoxLayout(self.centralwidget)
        main_layout.setContentsMargins(10, 8, 10, 8)
        main_layout.addWidget(self.tabWidget, 1)
        for widget in (
            self.btnCollectStart, self.btnCollectStop, self.label_3,
            self.label_RecordCount, self.label_4, self.label_State, self.label_9,
            self.txtCollectTimeSpan, self.label_10, self.txtFileSavePath,
        ):
            widget.hide()
        safety = QHBoxLayout()
        safety.addWidget(QLabel("软件停止不能替代硬件急停 · GELLO 无重力补偿，请托住 J2/J3"))
        safety.addStretch(1)
        safety.addWidget(self.gello_page_stop)
        safety.addWidget(self.gello_page_estop)
        main_layout.addLayout(safety)
        self.teleop_log.document().setMaximumBlockCount(2000)
        self.teleop_log.setFixedHeight(105)
        log_dock = QDockWidget("运行日志 · " + str(self.log_path), self)
        log_dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        log_dock.setWidget(self.teleop_log)
        self.addDockWidget(Qt.BottomDockWidgetArea, log_dock)
        self.setStyleSheet("""
            QPushButton { min-height: 28px; padding: 3px 10px; }
            QLineEdit, QComboBox, QDoubleSpinBox { min-height: 26px; }
            QGroupBox { font-weight: 600; margin-top: 12px; padding-top: 14px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; }
            QTabBar::tab { min-width: 135px; padding: 10px; }
        """)

    def _teleop_event(self, level, message):
        self.teleop_engine.events.put((level, message))

    def _run_teleop_async(self, name, function):
        running = self._teleop_async_threads.get(name)
        if running is not None and running.is_alive():
            self._teleop_event("warning", f"{name} 正在执行，请勿重复操作")
            return False

        def worker():
            try:
                function()
            except Exception as exc:
                self._teleop_event("error", f"{name}失败: {type(exc).__name__}: {exc}")

        thread = threading.Thread(target=worker, name=f"GUI-{name}", daemon=True)
        self._teleop_async_threads[name] = thread
        thread.start()
        return True

    def save_teleop_settings(self, _checked=False, show_event=True):
        try:
            starting = self._teleop_async_threads.get("启动跟随")
            if starting is not None and starting.is_alive():
                raise RuntimeError("跟随正在启动，请先停止跟随再修改参数")
            dataset = self.lerobot_recorder.snapshot()
            if (self.teleop_engine.state not in ("idle", "fault")
                    or dataset["episode_active"] or dataset["buffered_frames"]
                    or self._episode_operation_lock.locked()):
                raise RuntimeError("请先停止控制并处理当前 Episode，再修改参数")
            adapter = self.nrc_adapter
            if adapter is not None and getattr(adapter, "motion_mode", "servoj") != "servoj":
                raise RuntimeError(
                    "当前已连接控制通道仍是 MoveJ；请先停止并重启 ServoJ 实验副本"
                )
            runtime = {
                "robot": {
                    "ip": self.teleop_robot_ip.text().strip(),
                    "motion_mode": "servoj",
                    "movej_velocity": self.teleop_movej_velocity.value(),
                    "movej_acc": self.teleop_movej_acc.value(),
                    "movej_dec": self.teleop_movej_dec.value(),
                    "movej_period_s": self.teleop_movej_period.value(),
                    "movej_low_latency": self.teleop_movej_low_latency.isChecked(),
                    "movej_max_segment_deg": self.teleop_movej_segment.value(),
                    "movej_duplicate_deadband_deg": self.teleop_movej_duplicate_deadband.value(),
                    "servoj_vmax": self.teleop_servoj_vmax.value(),
                    "servoj_amax": self.teleop_servoj_amax.value(),
                    "servoj_jmax": self.teleop_servoj_jmax.value(),
                    "safety_max_command_step_rad": math.radians(self.gello_page_step_limit.value()),
                    "safety_max_tracking_error_rad": math.radians(self.gello_page_tracking_limit.value()),
                    "safety_max_command_speed_rad_s": self.gello_page_speed_limit.value(),
                },
                "gello": {
                    "port": self.teleop_gello_port.text().strip(),
                    "joint_scale": self.teleop_gello_joint_scale.value(),
                    "locked_joints": [
                        joint for joint, checkbox in self.teleop_locked_joint_checks.items()
                        if checkbox.isChecked()
                    ],
                },
                "o6": {
                    "port": self.teleop_o6_port.text().strip(),
                    "speed": [self.teleop_o6_speed.value()] * 6,
                },
                "preset": {
                    "robot_velocity_percent": self.teleop_preset_speed.value(),
                    "robot_acc_percent": self.teleop_preset_acc.value(),
                    "robot_dec_percent": self.teleop_preset_dec.value(),
                },
                "dataset": {
                    "root": self.lerobot_root_edit.text().strip(),
                    "task": self.lerobot_task_edit.text().strip(),
                    "camera_max_skew_s": self.dataset_skew_limit.value(),
                    "action_max_age_s": self.dataset_action_age_limit.value(),
                    "max_tracking_error_deg": self.dataset_tracking_limit.value(),
                },
                "shortcuts": self._collect_shortcut_values(),
            }
            self.teleop_store.update_runtime(runtime)
            # The adapter copies these values when connecting. Saving is stopped-only;
            # refresh its cached parameters before the next follow without any SDK call.
            robot_cfg = runtime["robot"]
            if adapter is not None:
                adapter.movej_velocity = robot_cfg["movej_velocity"]
                adapter.movej_acc = robot_cfg["movej_acc"]
                adapter.movej_dec = robot_cfg["movej_dec"]
                adapter.movej_period_s = robot_cfg["movej_period_s"]
                adapter.movej_low_latency = robot_cfg["movej_low_latency"]
            self.gello_controller.port = runtime["gello"]["port"]
            self.o6_controller.port = runtime["o6"]["port"]
            if self.o6_controller.connected:
                self.o6_controller.set_profile(
                    runtime["o6"]["speed"], self.teleop_store.data["o6"]["torque"]
                )
            self._apply_keyboard_shortcuts()
            self.collection_verified.setChecked(False)
            if show_event:
                self._teleop_event(
                    "info",
                    "参数已保存；ServoJ v/a/j="
                    f"{robot_cfg['servoj_vmax']:g}/{robot_cfg['servoj_amax']:g}/"
                    f"{robot_cfg['servoj_jmax']:g}，GELLO 反馈="
                    f"{self.teleop_store.data['gello']['feedback_hz']:g}Hz；"
                    "下次跟随生效，无需重连；本次未下发运动指令",
                )
            return True
        except Exception as exc:
            self._teleop_event("error", f"保存参数失败: {exc}")
            return False

    def MasterConnect(self):
        if any(
            thread is not None and thread.is_alive()
            for thread in (
                self._teleop_async_threads.get("连接主臂"),
                self._teleop_async_threads.get("连接主臂/O6"),
            )
        ):
            self._teleop_event("warning", "主臂连接正在执行，请勿重复操作")
            return
        if not self.save_teleop_settings(show_event=False):
            return
        self._run_teleop_async("连接主臂", self.teleop_engine.connect_master)

    def RoArmDevicesConnect(self):
        if any(
            thread is not None and thread.is_alive()
            for thread in (
                self._teleop_async_threads.get("连接主臂"),
                self._teleop_async_threads.get("连接主臂/O6"),
            )
        ):
            self._teleop_event("warning", "主臂连接正在执行，请勿重复操作")
            return
        if not self.save_teleop_settings(show_event=False):
            return
        self._run_teleop_async("连接主臂/O6", self.teleop_engine.connect_devices)

    def _camera_frames_ready(self):
        max_age = float(self.teleop_store.data["dataset"]["camera_max_age_s"])
        now = time.monotonic()
        with self._camera_frame_lock:
            return (
                self._wrist_rgb_frame is not None
                and self._base_rgb_frame is not None
                and self._base_roi_rgb_frame is not None
                and now - self._wrist_rgb_timestamp <= max_age
                and now - self._base_rgb_timestamp <= max_age
                and abs(self._base_rgb_timestamp - self._wrist_rgb_timestamp)
                <= float(self.teleop_store.data["dataset"]["camera_max_skew_s"])
            )

    def _workflow_connections_ready(self):
        return (
            self.robot1_connected
            and self.nrc_adapter is not None
            and self.master_controller.connected
            and self.o6_controller.connected
        )

    def _current_cr5_servo_state(self):
        snapshot = getattr(self, "_latest_robot_snapshot", None)
        if snapshot is None or snapshot[3] or time.monotonic() - self._latest_robot_snapshot_at > 1.0:
            return None
        return snapshot[2]

    def WorkflowPrepare(self, _checked=False):
        """Start both cameras and connect all devices without commanding motion."""
        if not self.save_teleop_settings(show_event=False):
            return
        self.CamerasStartAll()
        if not self.robot1_connected or self.nrc_adapter is None:
            self.RobotCONNECT()
        if not self.master_controller.connected or not self.o6_controller.connected:
            self.RoArmDevicesConnect()
        self._workflow_prepare_active = True
        self._workflow_prepare_started_at = time.monotonic()
        self._teleop_event(
            "info",
            "正在准备双相机、CR5、主臂和 O6（仅连接，不上使能、不启动跟随）",
        )

    def WorkflowPowerOn(self, _checked=False):
        """Power CR5 only; never start MoveJ follow or release the RoArm."""
        if self._workflow_power_starting:
            return
        if not self.robot1_connected or self.nrc_adapter is None:
            self._teleop_event("error", "CR5 尚未连接，无法上使能")
            return
        self._workflow_power_starting = True

        def power_only():
            try:
                transition = self.nrc_adapter.power_on()
                actions = "、".join(transition.actions) or "状态无需变化"
                self._teleop_event(
                    "info",
                    f"CR5 已单独上使能，伺服状态 "
                    f"{transition.initial_state}→{transition.final_state}：{actions}；"
                    "主从跟随尚未启动，主臂保持当前模式",
                )
                snapshot = getattr(self, "_latest_robot_snapshot", None)
                if snapshot is not None:
                    joints, tcp, _servo_state, _poll_error = snapshot
                    self._latest_robot_snapshot = (
                        joints,
                        tcp,
                        transition.final_state,
                        "",
                    )
            finally:
                self._workflow_power_starting = False

        if not self._run_teleop_async("CR5 上使能", power_only):
            self._workflow_power_starting = False

    def WorkflowStartTeleop(self, _checked=False):
        """Start GELLO MoveJ follow only; never connect devices or power the CR5."""
        if self.teleop_engine.master_type != "gello" or self.teleop_store.data["gello"]["control_mode"] != "joint":
            self._teleop_event("error", "本工作台只开放 GELLO 关节相对控制；实验映射不在本次数采范围内")
            return
        if self.teleop_engine.state == "following":
            self._teleop_event("info", "主从跟随已经启动")
            return
        if not self._workflow_connections_ready():
            self._teleop_event("error", "设备尚未全部连接，无法开始主从跟随")
            return
        # Do not gate an operator command on the 200 ms display cache.  The
        # runtime reads the controller's live servo state before starting ServoJ
        # and reports the exact boundary error if it is not running.
        self.TeleopFollowStart()

    def TeleopFollowStart(self):
        if not self.save_teleop_settings(show_event=False):
            return
        self.collection_verified.setChecked(False)
        self._run_teleop_async("启动跟随", self.teleop_engine.start_follow)

    def TeleopFollowStop(self):
        self._run_teleop_async(
            "停止跟随", lambda: self.teleop_engine.stop_follow("人工停止")
        )

    def TeleopStopCurrentAction(self):
        state = self.teleop_engine.state
        if state in ("following", "master_free"):
            operation = lambda: self.teleop_engine.stop_follow("人工停止当前动作")
        elif state in ("replay", "episode_replay"):
            operation = lambda: self.teleop_engine.stop_joint_replay(
                resume_follow=False
            )
        elif state in ("preset", "recovering"):
            operation = lambda: self.teleop_engine.emergency_stop(
                "用户中断当前高优先级动作"
            )
        else:
            self._teleop_event("warning", f"当前状态 {state} 没有可停止的动作")
            return
        self._run_teleop_async("停止当前动作", operation)

    def TeleopMasterFree(self):
        self._run_teleop_async(
            "从臂保持/主臂自由",
            self.teleop_engine.release_master_hold_slave,
        )

    def TeleopRecoverO6Motor(self):
        motor_number = int(self.teleop_o6_motor_combo.currentData())
        motor_name = O6_MOTOR_NAMES[motor_number - 1]
        answer = QMessageBox.warning(
            self,
            "确认恢复 O6 电机",
            f"将停止当前跟随，并尝试恢复 O6 {motor_number} 号电机（{motor_name}）。\n\n"
            "请确认夹持物和障碍已经移除，手指周围无人。恢复时电机仍可能轻微动作。",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return
        self._run_teleop_async(
            "恢复 O6 电机",
            lambda: self.teleop_engine.recover_o6_motor(motor_number),
        )

    def TeleopO6Action(self, action_name):
        self._run_teleop_async(
            "O6 快捷动作",
            lambda: self.teleop_engine.execute_o6_action(action_name),
        )

    def TeleopEnableM5Control(self):
        self._run_teleop_async(
            "恢复 M5 控制 O6",
            self.teleop_engine.enable_m5_o6_control,
        )

    def _set_replay_joint_selection(self, joint_indices):
        selected = set(joint_indices)
        for number, checkbox in self.teleop_replay_joint_checks.items():
            checkbox.setChecked(number in selected)

    def TeleopRecordStart(self, slot=1):
        if self.teleop_engine.state != "following" and not self.save_teleop_settings(show_event=False):
            return
        self._run_teleop_async(
            f"开始录制轨迹 {slot}",
            lambda: self.teleop_engine.start_joint_recording(slot),
        )

    def TeleopRecordStop(self, slot=1):
        self._run_teleop_async(
            f"停止录制轨迹 {slot}",
            lambda: self.teleop_engine.stop_joint_recording(slot),
        )

    def TeleopReplayStart(self, slot=1):
        insert_mode = bool(self.lerobot_recorder.snapshot()["episode_active"])
        # Replay is an intentional high-priority operator command.  Its button
        # and shortcut execute immediately; safety remains enforced by state,
        # feedback, speed and workspace checks plus the hardware E-stop.
        self._run_teleop_async(
            f"Episode 插入回放 {slot}" if insert_mode else f"启动回放 {slot}",
            lambda: self.teleop_engine.start_joint_replay(
                insert_into_episode=insert_mode,
                slot=slot,
            ),
        )

    def TeleopReplayStop(self):
        self._run_teleop_async(
            "停止关节轨迹重放",
            self.teleop_engine.stop_joint_replay,
        )

    def _lerobot_sample_provider(self):
        dataset_cfg = self.teleop_store.data["dataset"]
        sample = self.teleop_engine.dataset_sample()
        with self._camera_frame_lock:
            wrist_image = self._wrist_rgb_frame
            wrist_timestamp = self._wrist_rgb_timestamp
            base_image = self._base_rgb_frame
            roi_image = self._base_roi_rgb_frame
            base_timestamp = self._base_rgb_timestamp
        if wrist_image is None:
            raise RuntimeError("腕部 D435 尚无 RGB 图像，请先启动 CAMERA1")
        if base_image is None or roi_image is None:
            raise RuntimeError("基座 D435 尚无完整图/ROI 图像，请先启动 CAMERA2")
        now = time.monotonic()
        max_age = float(dataset_cfg["camera_max_age_s"])
        wrist_age = now - wrist_timestamp
        base_age = now - base_timestamp
        skew = abs(wrist_timestamp - base_timestamp)
        sample["quality"].update({
            "camera_skew_s": skew, "wrist_age_s": wrist_age, "base_age_s": base_age,
            "wrist_timestamp": wrist_timestamp, "base_timestamp": base_timestamp,
            "wrist_age_exceeded": float(wrist_age > max_age),
            "base_age_exceeded": float(base_age > max_age),
            "camera_skew_exceeded": float(
                skew > float(dataset_cfg["camera_max_skew_s"])
            ),
        })
        sample["image_base_rgb"] = base_image
        sample["image_wrist_rgb"] = wrist_image
        sample["image_roi_rgb"] = roi_image
        return sample

    def _run_episode_async(self, name, function):
        if not self._episode_operation_lock.acquire(blocking=False):
            self._teleop_event("warning", "另一个 Episode 操作正在执行，请等待完成")
            return
        self._episode_operation_name = name
        def guarded():
            try:
                function()
            finally:
                self._episode_operation_name = ""
                self._episode_operation_lock.release()

        if not self._run_teleop_async(name, guarded):
            self._episode_operation_name = ""
            self._episode_operation_lock.release()

    def LeRobotEpisodeStart(self):
        dataset = self.lerobot_recorder.snapshot()
        if dataset["episode_active"] or self._episode_operation_lock.locked():
            self._teleop_event("info", "LeRobot Episode 已在录制中")
            return
        task = self.lerobot_task_edit.text().strip()
        root = self.lerobot_root_edit.text().strip()
        try:
            # Only dataset text changes here. Saving all teleop parameters while
            # following used to rewrite O6 profiles and live safety settings.
            self.teleop_store.update_runtime({"dataset": {"task": task, "root": root}})
        except Exception as exc:
            self._teleop_event("error", f"保存采集参数失败：{exc}")
            return
        cfg = self.teleop_store.data
        context = copy.deepcopy({
            "application_directory": str(BASE_DIR),
            "hardware": {"controller": "NRC", "follower": "CR3A", "leader": "GELLO", "hand": "LinkerHand O6"},
            "robot": cfg["robot"], "gello": cfg["gello"], "o6": cfg["o6"],
            "dataset": cfg["dataset"],
            "operator_confirmed_hardware_follow": True,
            "camera_roles": {
                "base_0_rgb": "base camera full RGB",
                "left_wrist_0_rgb": "wrist camera full RGB",
                "right_wrist_0_rgb": "base camera ROI, not a third camera",
            },
            "action_source": "last SDK-sent target; not an execution acknowledgement",
        })
        self.lerobot_outcome.setCurrentIndex(0)
        self.lerobot_notes.clear()
        if dataset["buffered_frames"]:
            outcome = "failure" if dataset["error"] else "success"
            self._run_episode_async(
                "保存上一 Episode 并开始下一 Episode",
                lambda: (
                    self.lerobot_recorder.save_episode(outcome, "自动保存后开始下一条"),
                    self.lerobot_recorder.start_episode(task, root, metadata=context),
                ),
            )
        else:
            self._run_episode_async(
                "开始 LeRobot Episode",
                lambda: self.lerobot_recorder.start_episode(task, root, metadata=context),
            )

    def LeRobotEpisodeStop(self):
        self._run_episode_async("结束 Episode 录制", self.lerobot_recorder.stop_episode)

    def LeRobotEpisodeSave(self):
        snapshot = self.lerobot_recorder.snapshot()
        if snapshot["episode_active"]:
            self._teleop_event("error", "请先结束录制，再检查并保存")
            return
        outcome = self.lerobot_outcome.currentData()
        if outcome is None:
            outcome = "failure" if snapshot["error"] else "success"
        notes = self.lerobot_notes.text()
        self._run_episode_async(
            "保存 LeRobot Episode",
            lambda: self.lerobot_recorder.save_episode(outcome, notes),
        )

    def LeRobotEpisodeDiscard(self):
        snapshot = self.lerobot_recorder.snapshot()
        answer = QMessageBox.warning(
            self, "丢弃当前 Episode",
            f"将清除当前未保存的 {snapshot['buffered_frames']} 帧，不能撤销。\n"
            "已保存的 Episode 不受影响；不会自动开始下一条，也不会停止机械臂跟随。",
            QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return
        self._run_episode_async("丢弃 LeRobot Episode", self.lerobot_recorder.discard_episode)

    def LeRobotDatasetFinalize(self):
        snapshot = self.lerobot_recorder.snapshot()
        if snapshot["episode_active"] or snapshot["buffered_frames"]:
            self._teleop_event("error", "当前 Episode 未处理，请先结束录制并保存或丢弃；未清除任何数据")
            return
        self._run_episode_async("结束 LeRobot 数据集", self.lerobot_recorder.finalize)

    def TeleopSaveHome(self):
        dataset = self.lerobot_recorder.snapshot()
        if self.teleop_engine.state not in ("idle", "fault") or dataset["episode_active"] or dataset["buffered_frames"]:
            self._teleop_event("error", "请先停止跟随并处理 Episode，再保存 HOME")
            return
        self._run_teleop_async(
            "保存初始位 HOME",
            lambda: self.teleop_engine.save_preset("HOME"),
        )

    def TeleopGoHome(self):
        if self.teleop_engine.state not in ("idle", "fault", "following", "master_free"):
            self._teleop_event("error", "当前状态不允许回 HOME")
            return
        try:
            self.teleop_engine.recall_preset("HOME", resume_follow=False)
        except Exception as exc:
            self._teleop_event("error", f"主从臂回初始位失败: {exc}")

    def TeleopSavePreset(self, name):
        self._run_teleop_async(
            f"保存示教点 {name}", lambda: self.teleop_engine.save_preset(name)
        )

    def TeleopRecallPreset(self, name):
        try:
            self.teleop_engine.recall_preset(name, resume_follow=True)
        except Exception as exc:
            self._teleop_event("error", f"回到示教点 {name} 失败: {exc}")

    def EmergencyStop(self):
        self._run_teleop_async(
            "软件紧急停止", lambda: self.teleop_engine.emergency_stop("用户触发软件紧急停止")
        )

    def _collection_blockers(self, snapshot):
        """Operator preflight uses cached feedback only; no extra NRC requests."""
        cfg = self.teleop_store.data
        now = time.monotonic()
        reasons = []
        if snapshot["master_type"] != "gello" or cfg["gello"]["control_mode"] != "joint":
            reasons.append("数采只开放 GELLO 关节相对模式")
        if self.nrc_adapter is not None and self.nrc_adapter.controller_job_warning:
            reasons.append("控制柜报告仍有作业或状态异常，请先确认控制柜已就绪")
        if snapshot["state"] != "following":
            reasons.append("未启动关节跟随")
        if not self._workflow_connections_ready():
            reasons.append("CR3A / GELLO / O6 未全部连接")
        if self._current_cr5_servo_state() != 3:
            reasons.append("CR3A 使能状态未确认或反馈过期")
        master = snapshot["master"]
        if master is None or now - master.timestamp > cfg["gello"]["feedback_timeout_s"]:
            reasons.append("GELLO 反馈未就绪或过期")
        if (snapshot["o6_position"] is None
                or now - snapshot["o6_timestamp"] > cfg["o6"]["feedback_timeout_s"]):
            reasons.append("O6 反馈未就绪或过期")
        if snapshot["o6_fault"] is None or any(snapshot["o6_fault"]):
            reasons.append("O6 故障反馈未就绪或存在报警")
        if not self.D435_1_Started or not self.D435_2_Started or not self._camera_frames_ready():
            reasons.append("双相机画面未就绪、超时或接收时间差超限")
        telemetry = snapshot["gello_telemetry"]
        if telemetry is None or not telemetry["sent_count"]:
            reasons.append("等待实际反馈及已发目标")
        else:
            age_limit = cfg["dataset"]["action_max_age_s"]
            pending_target_deg = float(telemetry.get("pending_target_deg", 0.0))
            validated_at = float(telemetry.get("validated_at", telemetry["command_at"]))
            effective_action_at = (
                validated_at
                if pending_target_deg <= cfg["robot"]["movej_duplicate_deadband_deg"]
                else telemetry["command_at"]
            )
            if now - min(effective_action_at, telemetry["feedback_at"]) > age_limit:
                reasons.append("从臂反馈 / 指令流已过期")
            # 瞬时“已提交目标差”只做质量标记（保存时经 needs_review 标出），
            # 不能作为开始录制的硬门限——MoveJ 运动中它天然会短暂超过 5°。
            if telemetry.get("low_latency", False):
                if telemetry["desired_tracking_error_deg"] > cfg["dataset"]["max_tracking_error_deg"]:
                    reasons.append(f"完整期望误差超限：{telemetry['desired_tracking_error_deg']:.2f}°")
        if not self.lerobot_task_edit.text().strip():
            reasons.append("任务文字为空")
        if not self.lerobot_root_edit.text().strip():
            reasons.append("保存目录为空")
        return reasons

    def refresh_teleop_ui(self):
        snapshot = self.teleop_engine.snapshot()
        self._latest_teleop_snapshot = snapshot
        dataset = self.lerobot_recorder.snapshot()
        cfg = self.teleop_store.data
        now = time.monotonic()
        state = snapshot["state"]
        active = dataset["episode_active"]
        pending = dataset["buffered_frames"] > 0
        episode_busy = self._episode_operation_lock.locked()
        connecting = any(
            thread.is_alive() for name, thread in self._teleop_async_threads.items()
            if name in ("连接纳博特控制柜", "连接主臂", "连接主臂/O6")
        )
        starting_thread = self._teleop_async_threads.get("启动跟随")
        starting = starting_thread is not None and starting_thread.is_alive()
        moving = state not in ("idle", "fault", "closed")
        settings_locked = moving or starting or active or pending or episode_busy or snapshot["o6_action_active"]
        ready = self._workflow_connections_ready()
        controller_job_warning = (
            self.nrc_adapter.controller_job_warning
            if self.nrc_adapter is not None else ""
        )
        servo = self._current_cr5_servo_state()
        servo_text = {0: "停止", 1: "未使能", 2: "报警", 3: "已使能"}.get(servo, "未知/过期")
        state_name = {
            "idle": "等待操作", "following": "关节指令流运行（查看实际反馈）",
            "fault": "故障停止", "preset": "HOME / 示教点回位中",
            "recovering": "O6 恢复中", "master_free": "从臂保持",
            "closed": "已关闭",
        }.get(state, state)
        color = "#b42318" if state == "fault" else "#175d7a"
        self.gello_page_state_label.setText(f"遥操：{state_name}")
        self.gello_page_state_label.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {color};")
        self.gello_page_device_label.setText(
            f"CR3A {'已连接' if self.robot1_connected else '未连接'} / {servo_text}  ·  "
            f"GELLO {'已连接' if self.master_controller.connected else '未连接'}  ·  "
            f"O6 {'已连接' if self.o6_controller.connected else '未连接'}"
        )
        self.connection_mode_label.setText(
            f"{cfg['robot']['motion_mode']} / {cfg['gello']['control_mode']} / robot_num={cfg['robot']['robot_num']}"
            f" / J1-J6倍率={cfg['gello']['joint_scale']:.2f}×"
            f" / 锁定关节={cfg['gello']['locked_joints'] or '无'}；实验 TCP 模式未开放数采"
        )
        master = snapshot["master"]
        master_fresh = master is not None and now - master.timestamp <= cfg["gello"]["feedback_timeout_s"]
        self.gello_page_joint_label.setText(
            f"J1-J6 倍率 {cfg['gello']['joint_scale']:.2f}×  ·  J7 开合比例 {master.gripper:.3f}  ·  "
            f"O6 反馈 {snapshot['o6_position']}  ·  故障 {snapshot['o6_fault']}"
            if master_fresh and snapshot["master_type"] == "gello"
            else "GELLO 反馈未就绪或已过期"
        )
        telemetry = snapshot["gello_telemetry"]
        actual = None
        target = None
        desired = None
        validated = None
        if telemetry is not None:
            actual = telemetry["actual_deg"]
            target = telemetry["target_deg"]
            desired = telemetry.get("desired_deg")
            validated = telemetry.get("validated_deg", target)
            feedback_age = now - telemetry["feedback_at"]
            validated_age = now - telemetry.get("validated_at", telemetry["command_at"])
            submit_age = now - telemetry["command_at"]
            pending_target_deg = float(telemetry.get("pending_target_deg", 0.0))
            latency = telemetry.get("latency")
            submit_hz = latency["recent_send_hz"] if latency is not None else telemetry["send_hz"]
            rate_text = (
                f"GELLO 反馈 {cfg['gello']['feedback_hz']:g} Hz"
                if cfg["robot"]["motion_mode"] == "servoj"
                else f"指令周期 {cfg['robot']['movej_period_s']:g}s"
            )
            self.gello_stream_label.setText(
                f"{cfg['robot']['motion_mode']} SDK 提交 {telemetry['sent_count']} 帧（不等于执行确认）  ·  "
                f"最近提交 {submit_hz:.1f} Hz / {rate_text}  ·  "
                f"已提交目标差 {telemetry['tracking_error_deg']:.2f}°  ·  "
                f"反馈年龄 {feedback_age * 1000:.0f} ms  ·  "
                f"有效目标年龄 {validated_age * 1000:.0f} ms  ·  "
                f"实际提交年龄 {submit_age * 1000:.0f} ms  ·  "
                f"待提交差 {pending_target_deg:.2f}°"
                + ("  [已过期]" if feedback_age > cfg["dataset"]["action_max_age_s"] else "")
            )
            self.gello_latency_label.setText(
                f"软件时序 · {latency['phase']} · GELLO 样本年龄 {latency['leader_age_ms']:.0f}ms · "
                f"关节读取 {latency['feedback_read_ms']:.0f}ms · 忙检查 {latency['busy_check_ms']:.0f}ms · "
                f"发送调用 {latency['dispatch_ms']:.0f}ms\n"
                f"忙等待 {latency['busy_wait_ms']:.0f}ms / 上次 {latency['last_busy_wait_ms']:.0f}ms · "
                f"本次目标跨度 {latency['segment_delta_deg']:.2f}° · "
                f"待提交差 {pending_target_deg:.2f}° · "
                f"完整期望差 {telemetry['desired_tracking_error_deg']:.2f}°"
                if latency is not None else "ServoJ 软件时序：尚未测量"
            )
        else:
            robot_snapshot = getattr(self, "_latest_robot_snapshot", None)
            if robot_snapshot is not None and not robot_snapshot[3] and now - self._latest_robot_snapshot_at < 1.0:
                actual = robot_snapshot[0]
            self.gello_stream_label.setText("指令流尚未产生反馈；SDK 返回成功不代表运动已执行")
            self.gello_latency_label.setText("ServoJ 软件时序：尚未测量")
        for row in range(6):
            leader = math.degrees(master.arm_joints_rad[row]) if master_fresh and snapshot["master_type"] == "gello" else None
            values = (leader, actual[row] if actual is not None else None,
                      desired[row] if desired is not None else None,
                      validated[row] if validated is not None else None,
                      target[row] if target is not None else None,
                      desired[row] - actual[row] if desired is not None and actual is not None else None)
            for column, value in enumerate(values, 1):
                self.gello_joint_table.item(row, column).setText("--" if value is None else f"{value:.2f}")
        if state != "following":
            self.collection_verified.setChecked(False)
        self.collection_verified.setEnabled(state == "following" and not active and not pending and not episode_busy)
        reasons = self._collection_blockers(snapshot)
        self.collection_preflight.setText(
            "采集提示：" + "；".join(reasons[:2])
            + (f"（另 {len(reasons) - 2} 项，悬停查看）" if len(reasons) > 2 else "")
            if reasons else "采集提示：当前状态信息正常"
        )
        self.collection_preflight.setToolTip("\n".join(reasons))
        self.lerobot_start_button.setToolTip("开始记录；已停止的上一 Episode 会自动保存后开始下一条")
        self.collection_preflight.setStyleSheet("color: #985500;" if reasons else "color: #167044;")
        self.workflow_status_label.setText(
            f"控制状态：{state_name}" + (f"\n{snapshot['error']}" if snapshot["error"] else "")
            + "\n开始 → 结束录制 → 标注并保存；结束录制不停止跟随，停止运动请按 F7。"
        )
        self.gello_page_camera_label.setText(
            f"双相机：{'画面就绪' if self.D435_1_Started and self.D435_2_Started and self._camera_frames_ready() else '未就绪'}"
            "  ·  三路预览在数采工作台"
        )
        if episode_busy:
            dataset_text = f"{self._episode_operation_name}，请等待；停止运动按钮仍可用"
        elif active:
            dataset_text = f"● 录制中 · {dataset['buffered_frames']} 帧 / {dataset['duration']:.1f}s"
        elif dataset["error"]:
            dataset_text = "采集出现问题；可选择任务失败后保存已有帧，或丢弃：" + dataset["error"]
        elif pending:
            dataset_text = f"待审 Episode · {dataset['buffered_frames']} 帧；请选择任务结果后保存或丢弃"
        else:
            dataset_text = f"未录制 · 本次数据集已保存 {dataset['saved_episodes']} 条"
        self.gello_page_lerobot_label.setText(dataset_text)
        self.lerobot_status_label.setText(dataset_text)
        self.lerobot_status_label.setToolTip(dataset["root"])
        q = dataset["quality"] if active or pending or not dataset["last_saved"] else dataset["last_saved"]["quality"]
        self.collection_quality_label.setText(
            f"实际采样 {q['effective_fps']:.1f} / {q['configured_fps']} FPS  ·  "
            f"延迟帧 {q['late_frames']}  ·  最大间隔 {q['max_frame_gap_s'] * 1000:.0f} ms  ·  "
            f"最大跟踪误差 {q['max'].get('tracking_error_deg', 0):.2f}°  ·  "
            f"相机最大时间差 {q['max'].get('camera_skew_s', 0) * 1000:.0f} ms  ·  "
            f"重复画面 腕/基={q['reused_camera_frames']['wrist']}/{q['reused_camera_frames']['base']}"
        )
        record_slot = snapshot["record_slot"]
        for slot_info in snapshot["replay_slots"]:
            slot = slot_info["slot"]
            status = "录制中" if snapshot["recording"] and slot == record_slot else slot_info["file_status"]
            self.teleop_replay_slot_status_labels[slot].setText(f"轨迹{slot}：{status}")
            self.teleop_record_start_buttons[slot].setEnabled(
                ready and state == "following" and not snapshot["recording"]
                and not snapshot["replaying"] and not episode_busy
            )
            self.teleop_record_stop_buttons[slot].setEnabled(
                snapshot["recording"] and slot == record_slot and not episode_busy
            )
            self.teleop_replay_start_buttons[slot].setEnabled(
                slot_info["available"] and not snapshot["recording"]
                and not snapshot["replaying"] and state in ("idle", "following")
                and not episode_busy
            )
        self.teleop_replay_stop_button.setEnabled(snapshot["replaying"] and not episode_busy)
        self.teleop_replay_status_label.setText(
            "正在重放动作轨迹" if snapshot["replaying"] else "动作轨迹可记录/重放；LeRobot Episode 可连续保存多条"
        )
        self.lerobot_start_button.setEnabled(not active and not episode_busy)
        self.lerobot_stop_button.setEnabled(active and not episode_busy)
        self.lerobot_save_button.setEnabled(
            pending and not active and not episode_busy
            and (self.lerobot_outcome.currentData() is not None or bool(dataset["error"]))
        )
        self.lerobot_discard_button.setEnabled((pending or active or bool(dataset["error"])) and not episode_busy)
        self.lerobot_finalize_button.setEnabled(dataset["session_active"] and not active and not pending and not episode_busy)
        self.lerobot_task_edit.setEnabled(not active and not pending and not episode_busy)
        self.lerobot_root_edit.setEnabled(not dataset["session_active"] and not episode_busy)
        self.lerobot_outcome.setEnabled(pending and not active and not episode_busy)
        self.lerobot_notes.setEnabled(pending and not active and not episode_busy)
        self.maintenance_page.setEnabled(not settings_locked)
        self.gello_page_connect_robot.setEnabled(not self.robot1_connected and not connecting and not settings_locked)
        self.gello_page_connect_devices.setEnabled(not ready and not connecting and not settings_locked)
        self.gello_read_only_button.setEnabled(not self.master_controller.connected and not connecting and not settings_locked)
        self.gello_page_power_on.setEnabled(
            self.robot1_connected and servo in (0, 1) and not settings_locked and not self._workflow_power_starting
        )
        self.gello_page_start.setEnabled(
            ready and not settings_locked and not connecting and state != "closed"
        )
        self.gello_page_start.setToolTip(
            "控制柜报告仍有作业或状态异常；请先确认控制柜已就绪"
            if controller_job_warning else "以当前主从姿态为零点开始 GELLO 关节跟随"
        )
        self.gello_page_stop.setEnabled(moving or starting)
        self.gello_page_estop.setEnabled(True)
        self.gello_page_save_home.setEnabled(ready and not settings_locked)
        self.gello_page_go_home.setEnabled(ready and "HOME" in snapshot["presets"] and not settings_locked)
        self.workflow_home_button.setEnabled(
            ready
            and "HOME" in snapshot["presets"]
            and state in ("idle", "fault", "following", "master_free")
            and not episode_busy
        )
        self.gello_page_clear_error.setEnabled(self.robot1_connected and not self._workflow_power_starting)
        self.workflow_prepare_button.setEnabled(not settings_locked and not connecting and not self._workflow_prepare_active)
        self.teleop_robot_ip.setReadOnly(self.robot1_connected or connecting)
        self.teleop_gello_port.setReadOnly(self.master_controller.connected or connecting)
        self.teleop_o6_port.setReadOnly(self.o6_controller.connected or connecting)
        self.teleop_o6_recover_button.setEnabled(self.o6_controller.connected and not snapshot["o6_action_active"])
        self.teleop_o6_mode_label.setText("跟随时由 GELLO J7 独占控制；此处动作仅用于停止后的检查")
        for button in self.teleop_o6_action_buttons.values():
            button.setEnabled(self.o6_controller.connected and not snapshot["o6_action_active"])
        wrist = self.D435_1_Started
        base = self.D435_2_Started
        with self._camera_frame_lock:
            wrist_age = now - self._wrist_rgb_timestamp if self._wrist_rgb_timestamp else None
            base_age = now - self._base_rgb_timestamp if self._base_rgb_timestamp else None
        self.camera_status_label.setText(
            f"接收年龄：腕 {f'{wrist_age * 1000:.0f} ms' if wrist and wrist_age is not None else '--'}"
            f" / 基座 {f'{base_age * 1000:.0f} ms' if base and base_age is not None else '--'}"
            "（软件时间戳）"
        )
        self.camera_start_all_button.setEnabled(not (wrist and base) and not active and not episode_busy)
        self.camera_stop_all_button.setEnabled((wrist or base) and not active and not episode_busy)
        self.btnD435_1_Start.setEnabled(not wrist)
        self.btnD435_1_Stop.setEnabled(wrist)
        self.btnD435_2_Start.setEnabled(not base)
        self.btnD435_2_Stop.setEnabled(base)
        self.btnD435_1_Cut.setEnabled(wrist and self._wrist_rgb_frame is not None)
        self.btnD435_2_Cut.setEnabled(base and self._base_rgb_frame is not None)
        if self._workflow_prepare_active:
            if ready and self._camera_frames_ready() or now - self._workflow_prepare_started_at > 45.0:
                self._workflow_prepare_active = False
                self._teleop_event(
                    "info" if ready and self._camera_frames_ready() else "error",
                    "准备流程结束；请按页面反馈逐项检查，不会自动启动跟随",
                )
        while True:
            try:
                level, message = self.teleop_engine.events.get_nowait()
            except queue.Empty:
                break
            self._file_logger.log(getattr(logging, level.upper()), message)
            self.teleop_log.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {level.upper()}: {message}")
            self.statusBar().showMessage(message, 10000 if level == "error" else 5000)

    def keyPressEvent(self, event):
        # All operator keys are handled by configurable QShortcut objects.
        super().keyPressEvent(event)

    def eventFilter(self, watched, event):
        if isinstance(watched, QKeySequenceEdit):
            if event.type() == QEvent.FocusIn:
                for action_id, shortcut in self._keyboard_shortcuts.items():
                    shortcut.setEnabled(action_id == "emergency_stop")
            elif event.type() == QEvent.FocusOut:
                QTimer.singleShot(0, self._restore_keyboard_shortcuts_after_edit)
        return super().eventFilter(watched, event)

    def _restore_keyboard_shortcuts_after_edit(self):
        if isinstance(QApplication.focusWidget(), QKeySequenceEdit):
            return
        for shortcut in self._keyboard_shortcuts.values():
            shortcut.setEnabled(True)


    # ROBOTIQ ForceSensor.................................................................
    def ForceSensor_1_INIT(self):
        # 按钮点击之后，启动子线程（否则界面卡死）
        self.my_thread_ForceSensorCollect = mythread_ForceSensor()
        self.my_thread_ForceSensorCollect.start()

    def update_frame_ForceSensor_1(self):
        global ForceSensorData
        ForceSensorData = Sensor_Robotiq.ForceSensorValue
        ForceSensorDataStr = str(round(ForceSensorData[0], 2)) + "," + str(round(ForceSensorData[1], 2)) + "," + str(round(ForceSensorData[2], 2)) + "," + str(round(ForceSensorData[3], 2)) + "," + str(round(ForceSensorData[4], 2)) + "," + str(round(ForceSensorData[5], 2))
        self.txtForceSensorValue.setPlainText(ForceSensorDataStr)


    # D435 CAMERA....................................................................
    def _setup_d435_2_ui(self):
        """GELLO-only operator navigation; legacy backends remain internal."""
        self.tabWidget_2.hide()
        root = QVBoxLayout(self.tab_4)
        root.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        self.camera_console_content = QWidget()
        scroll.setWidget(self.camera_console_content)
        console = QVBoxLayout(self.camera_console_content)
        console.setContentsMargins(14, 12, 14, 12)
        root.addWidget(scroll)

        self.workflow_status_label = QLabel("先在 GELLO 控制页检查从臂实际响应，再采集。")
        self.workflow_status_label.setWordWrap(True)
        prepare_row = QHBoxLayout()
        prepare_row.addWidget(self.workflow_status_label, 1)
        self.workflow_prepare_button = QPushButton("准备相机与设备（不启动跟随） [F5]")
        self.workflow_prepare_button.clicked.connect(self.WorkflowPrepare)
        prepare_row.addWidget(self.workflow_prepare_button)
        console.addLayout(prepare_row)
        self.workflow_power_button = self.gello_page_power_on
        self.workflow_follow_button = self.gello_page_start
        self.workflow_stop_button = self.gello_page_stop
        self.workflow_estop_button = self.gello_page_estop
        self.workflow_home_button = QPushButton("一键回 HOME")
        self.workflow_home_button.setToolTip(
            "直接接替当前 GELLO 遥操作：先停止跟随，再执行已保存的 HOME 回位；"
            "录制中的 Episode 会持续记录 HOME 回位过程。"
        )
        self.workflow_home_button.clicked.connect(self.TeleopGoHome)
        self.workflow_clear_error_button = self.gello_page_clear_error
        prepare_row.addWidget(self.workflow_home_button)

        dataset_layout = self.teleop_dataset_group.layout()
        while dataset_layout.count():
            item = dataset_layout.takeAt(0)
            if item.widget() is not None:
                item.widget().hide()
        self.collection_verified = QCheckBox(
            "本次跟随已在低速下目视确认：CR3A 实际响应正常，J7 可控制 O6"
        )
        self.collection_verified.setToolTip(
            "仅本次跟随有效，不保存到配置；故障或重新启动跟随后需要重新确认。"
        )
        self.collection_verified.toggled.connect(
            lambda checked: setattr(self, "_collection_accepted", checked)
        )
        self.collection_verified.setChecked(True)
        self.collection_verified.hide()
        self.collection_preflight = QLabel("采集前检查：尚未就绪")
        self.collection_preflight.setWordWrap(True)
        self.lerobot_stop_button = QPushButton("结束录制 · 待审")
        self.lerobot_stop_button.clicked.connect(self.LeRobotEpisodeStop)
        self.lerobot_outcome = QComboBox()
        self.lerobot_outcome.addItem("请选择任务结果", None)
        self.lerobot_outcome.addItem("成功示范", "success")
        self.lerobot_outcome.addItem("任务失败（保留有效数据）", "failure")
        self.lerobot_notes = QLineEdit()
        self.lerobot_notes.setPlaceholderText("可选：物体、场景、失败原因等")
        self.collection_quality_label = QLabel("实际 FPS -- · 帧数 0 · 时长 0 s")
        self.collection_quality_label.setWordWrap(True)
        dataset_layout.addWidget(QLabel("任务指令"), 0, 0)
        dataset_layout.addWidget(self.lerobot_task_edit, 0, 1, 1, 4)
        dataset_layout.addWidget(QLabel("保存目录"), 1, 0)
        dataset_layout.addWidget(self.lerobot_root_edit, 1, 1, 1, 4)
        dataset_layout.addWidget(self.collection_preflight, 3, 0, 1, 5)
        for column, button in enumerate((
            self.lerobot_start_button, self.lerobot_stop_button,
            self.lerobot_save_button, self.lerobot_discard_button,
            self.lerobot_finalize_button,
        )):
            dataset_layout.addWidget(button, 4, column)
            button.show()
        dataset_layout.addWidget(self.lerobot_outcome, 5, 0, 1, 2)
        dataset_layout.addWidget(self.lerobot_notes, 5, 2, 1, 3)
        dataset_layout.addWidget(self.lerobot_status_label, 6, 0, 1, 5)
        dataset_layout.addWidget(self.collection_quality_label, 7, 0, 1, 5)
        for widget in (self.lerobot_task_edit, self.lerobot_root_edit, self.lerobot_status_label):
            widget.show()
        self.lerobot_save_button.setText("保存当前 [F9]")
        self.lerobot_discard_button.setText("丢弃当前 [F10]")
        self.lerobot_start_button.setText("开始 Episode [F8]")
        self.lerobot_save_button.setToolTip("标注结果后保存；保存完成后可再次点击“开始 Episode”录制下一条")
        self.lerobot_save_button.setStyleSheet(
            "QPushButton:enabled { background: #dff4e5; color: #17542c; }"
            "QPushButton:disabled { color: #999; }"
        )
        console.addWidget(self.teleop_dataset_group)
        console.addWidget(self.teleop_replay_group)

        camera_group = QGroupBox("训练图像预览 · 两台相机 / 三路 RGB")
        camera_layout = QVBoxLayout(camera_group)
        controls = QHBoxLayout()
        self.camera_start_all_button = QPushButton("启动双相机")
        self.camera_stop_all_button = QPushButton("停止双相机")
        self.camera_start_all_button.clicked.connect(self.CamerasStartAll)
        self.camera_stop_all_button.clicked.connect(self.CamerasStopAll)
        controls.addWidget(self.camera_start_all_button)
        controls.addWidget(self.camera_stop_all_button)
        self.camera_status_label = QLabel("腕部 -- | 基座 --")
        self.camera_status_label.setWordWrap(True)
        controls.addWidget(self.camera_status_label, 1)
        camera_layout.addLayout(controls)
        previews = QHBoxLayout()
        self.D435_2_Pic_Color = QLabel("基座 RGB 尚未启动")
        self.D435_2_Pic_ROI = QLabel("基座 ROI 尚未启动")
        self.D435_Pic_Color.setText("腕部 RGB 尚未启动")
        for title, label in (
            ("base_0 · 基座全图", self.D435_2_Pic_Color),
            ("left_wrist_0 · 腕部全图", self.D435_Pic_Color),
            ("right_wrist_0 · 基座 ROI（非第三台相机）", self.D435_2_Pic_ROI),
        ):
            card = QGroupBox(title)
            card_layout = QVBoxLayout(card)
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumSize(240, 160)
            label.setMaximumHeight(210)
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            label.setStyleSheet("background: #18232e; color: #d6e1eb;")
            card_layout.addWidget(label)
            previews.addWidget(card)
        camera_layout.addLayout(previews)
        self.D435_2_Status = QLabel(
            f"224×224 RGB · state 18D / action 12D · ROI {list(self.base_roi_norm)}"
        )
        self.D435_2_Status.setWordWrap(True)
        camera_layout.addWidget(self.D435_2_Status)
        console.addWidget(camera_group)
        console.addStretch(1)

        self.maintenance_page = QScrollArea()
        self.maintenance_page.setWidgetResizable(True)
        maintenance_content = QWidget()
        self.maintenance_page.setWidget(maintenance_content)
        settings = QVBoxLayout(maintenance_content)
        settings.setContentsMargins(16, 12, 16, 12)
        connection_group = QGroupBox("连接参数 · 保存后生效，设备连接期间不可改端口")
        form = QFormLayout(connection_group)
        form.addRow("控制柜 IP（6001 / 7000）", self.teleop_robot_ip)
        form.addRow("GELLO 串口", self.teleop_gello_port)
        form.addRow("O6 串口（Modbus 39）", self.teleop_o6_port)
        self.connection_mode_label = QLabel()
        form.addRow("控制路径", self.connection_mode_label)
        settings.addWidget(connection_group)
        settings.addWidget(self.gello_limits_group)

        quality_group = QGroupBox("采集质量门限 · 只停止采集，不改变运动安全限位")
        quality_form = QFormLayout(quality_group)
        cfg = self.teleop_store.data["dataset"]
        self.dataset_skew_limit = self._make_double_spin(
            cfg["camera_max_skew_s"], 0.01, 1.0, 0.01, 2, " s"
        )
        self.dataset_action_age_limit = self._make_double_spin(
            cfg["action_max_age_s"], 0.05, 1.0, 0.05, 2, " s"
        )
        self.dataset_tracking_limit = self._make_double_spin(
            cfg["max_tracking_error_deg"], 0.1, 20.0, 0.5, 1, " °"
        )
        quality_form.addRow("双相机主机接收时间差 ≤", self.dataset_skew_limit)
        quality_form.addRow("从臂反馈 / 已发目标年龄 ≤", self.dataset_action_age_limit)
        quality_form.addRow("记录时关节跟踪误差 ≤", self.dataset_tracking_limit)
        note = QLabel(
            "这些是软件采集检查，不代表硬件同步或 ServoJ 实机验收通过。"
            "超限 Episode 保留错误信息，不能作为合格数据保存。"
        )
        note.setWordWrap(True)
        quality_form.addRow(note)
        settings.addWidget(quality_group)

        scale_group = QGroupBox("GELLO → CR3A 关节映射幅度")
        scale_form = QFormLayout(scale_group)
        self.teleop_gello_joint_scale = self._make_double_spin(
            self.teleop_store.data["gello"]["joint_scale"], 0.5, 1.5, 0.05, 2, " ×"
        )
        scale_form.addRow("J1-J6 全局相对倍率", self.teleop_gello_joint_scale)
        scale_note = QLabel(
            "1.10× 表示 GELLO 相对变化放大 10%；仅作用于 J1-J6，不改变 J7/O6。\n"
            "倍率同时进入目标步长和速度保护；提高倍率前请确认 CR3A 远离关节限位。"
        )
        scale_note.setWordWrap(True)
        scale_form.addRow(scale_note)
        self.teleop_locked_joint_checks = {}
        locked_joint_row = QWidget()
        locked_joint_layout = QHBoxLayout(locked_joint_row)
        locked_joint_layout.setContentsMargins(0, 0, 0, 0)
        for joint_number in range(1, 7):
            checkbox = QCheckBox(f"J{joint_number}")
            checkbox.setChecked(joint_number in self.teleop_store.data["gello"]["locked_joints"])
            checkbox.setToolTip("勾选即锁定该 CR3A 关节；取消勾选即允许 GELLO 控制")
            self.teleop_locked_joint_checks[joint_number] = checkbox
            locked_joint_layout.addWidget(checkbox)
        locked_joint_layout.addStretch(1)
        scale_form.addRow("锁定 CR3A 关节", locked_joint_row)
        lock_note = QLabel(
            "勾选的关节固定为开始跟随时的 CR3A 姿态，同时不参与 GELLO 目标和速度映射。"
        )
        lock_note.setWordWrap(True)
        scale_form.addRow(lock_note)
        settings.addWidget(scale_group)

        speed_group = QGroupBox("GELLO 从臂跟随参数（ServoJ）")
        speed_form = QFormLayout(speed_group)
        self.teleop_movej_low_latency = QCheckBox("启用短段 MoveJ 低延迟实验")
        self.teleop_movej_low_latency.setChecked(self.teleop_store.data["robot"]["movej_low_latency"])
        self.teleop_movej_segment = self._make_double_spin(
            self.teleop_store.data["robot"]["movej_max_segment_deg"], 0.1, 5.0, 0.1, 1, " °"
        )
        self.teleop_movej_segment.setEnabled(self.teleop_movej_low_latency.isChecked())
        self.teleop_movej_low_latency.toggled.connect(self.teleop_movej_segment.setEnabled)
        self.teleop_movej_velocity = self._make_double_spin(
            self.teleop_store.data["robot"]["movej_velocity"], 1.0, 100.0, 5.0, 1, " %"
        )
        self.teleop_movej_acc = self._make_double_spin(
            self.teleop_store.data["robot"]["movej_acc"], 1.0, 100.0, 5.0, 1, " %"
        )
        self.teleop_movej_dec = self._make_double_spin(
            self.teleop_store.data["robot"]["movej_dec"], 1.0, 100.0, 5.0, 1, " %"
        )
        self.teleop_movej_period = self._make_double_spin(
            self.teleop_store.data["robot"]["movej_period_s"], 0.05, 0.5, 0.01, 2, " s"
        )
        self.teleop_movej_duplicate_deadband = self._make_double_spin(
            self.teleop_store.data["robot"]["movej_duplicate_deadband_deg"],
            0.0, 1.0, 0.01, 2, " °",
        )
        self.teleop_servoj_vmax = self._make_double_spin(
            self.teleop_store.data["robot"]["servoj_vmax"], 1.0, 200.0, 5.0, 1, ""
        )
        self.teleop_servoj_amax = self._make_double_spin(
            self.teleop_store.data["robot"]["servoj_amax"], 1.0, 500.0, 5.0, 1, ""
        )
        self.teleop_servoj_jmax = self._make_double_spin(
            self.teleop_store.data["robot"]["servoj_jmax"], 1.0, 1000.0, 10.0, 1, ""
        )
        speed_form.addRow("ServoJ vmax", self.teleop_servoj_vmax)
        speed_form.addRow("ServoJ amax", self.teleop_servoj_amax)
        speed_form.addRow("ServoJ jmax", self.teleop_servoj_jmax)
        speed_note = QLabel(
            "当前 GELLO 跟随固定使用 NRC 7000 ServoJ；ServoJ 会按 GELLO 反馈频率连续发送目标。\n"
            "先停止跟随再调节，点击“保存并应用参数”，下次开始跟随生效。"
        )
        speed_note.setWordWrap(True)
        speed_form.addRow(speed_note)
        settings.addWidget(speed_group)
        home_group = QGroupBox("HOME 回位 / O6 参数")
        home_form = QFormLayout(home_group)
        home_form.addRow("HOME 速度 %", self.teleop_preset_speed)
        home_form.addRow("HOME 加速度 %", self.teleop_preset_acc)
        home_form.addRow("HOME 减速度 %", self.teleop_preset_dec)
        home_form.addRow("O6 速度 / 255", self.teleop_o6_speed)
        settings.addWidget(home_group)
        settings.addWidget(self.teleop_save_settings_button)
        settings.addWidget(self.gello_page_clear_error)
        settings.addWidget(self.teleop_recovery_group)
        settings.addWidget(self.teleop_o6_action_group)
        self.teleop_o6_m5_button.hide()
        self.teleop_o6_action_group.setTitle("O6 单独检查 · 必须先停止 GELLO 跟随")
        settings.addWidget(self._build_shortcut_editor_group())

        # Per-camera diagnostic controls are kept in maintenance, not mixed
        # with episode buttons. No depth stream is used by this workspace.
        camera_checks = QGroupBox("相机单独检查")
        checks_layout = QHBoxLayout(camera_checks)
        self.btnD435_2_Start = QPushButton("启动基座")
        self.btnD435_2_Stop = QPushButton("停止基座")
        self.btnD435_2_Cut = QPushButton("基座 / ROI 截图")
        for widget, title in (
            (self.btnD435_1_Start, "启动腕部"), (self.btnD435_1_Stop, "停止腕部"),
            (self.btnD435_1_Cut, "腕部截图"),
        ):
            widget.setText(title)
        for widget in (
            self.btnD435_1_Start, self.btnD435_1_Stop, self.btnD435_1_Cut,
            self.btnD435_2_Start, self.btnD435_2_Stop, self.btnD435_2_Cut,
        ):
            checks_layout.addWidget(widget)
        settings.addWidget(camera_checks)
        settings.addStretch(1)
        self.tabWidget.clear()
        self.tabWidget.addTab(self.gello_page, "GELLO 控制")
        self.tabWidget.addTab(self.tab_4, "数采工作台")
        self.tabWidget.addTab(self.maintenance_page, "参数维护")
        self.tabWidget.setCurrentWidget(self.gello_page)
        self._apply_keyboard_shortcuts()

    def _build_shortcut_editor_group(self):
        group = QGroupBox("键盘快捷键绑定")
        outer = QVBoxLayout(group)
        note = QLabel(
            "点击快捷键输入框后直接按组合键；清空表示禁用。"
            "光标在普通文字/数字输入框中时不会执行快捷功能；"
            "编辑快捷键时会暂停其他绑定，软件紧急停止仍保留。"
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #555;")
        outer.addWidget(note)
        tabs = QTabWidget()

        def make_page(action_ids):
            page = QWidget()
            layout = QGridLayout(page)
            for index, action_id in enumerate(action_ids):
                editor = QKeySequenceEdit(
                    QKeySequence(self.teleop_store.data["shortcuts"][action_id])
                )
                editor.setToolTip("点击后按下新的按键组合；按 Backspace 可清空")
                editor.installEventFilter(self)
                self.shortcut_editors[action_id] = editor
                row = index // 2
                column = (index % 2) * 2
                layout.addWidget(QLabel(SHORTCUT_LABELS[action_id]), row, column)
                layout.addWidget(editor, row, column + 1)
            layout.setColumnStretch(1, 1)
            layout.setColumnStretch(3, 1)
            return page

        tabs.addTab(make_page(COMMON_SHORTCUT_IDS), "常用操作")
        outer.addWidget(tabs)
        controls = QHBoxLayout()
        self.shortcut_save_button = QPushButton("保存并立即应用快捷键")
        self.shortcut_save_button.clicked.connect(self.save_keyboard_shortcuts)
        self.shortcut_status_label = QLabel("快捷键尚未修改")
        controls.addWidget(self.shortcut_save_button)
        controls.addWidget(self.shortcut_status_label, 1)
        outer.addLayout(controls)
        return group

    def _collect_shortcut_values(self):
        values = {}
        assigned = {}
        for action_id, editor in self.shortcut_editors.items():
            sequence = editor.keySequence().toString(QKeySequence.PortableText).strip()
            values[action_id] = sequence
            if not sequence:
                continue
            normalized = sequence.casefold()
            if normalized in assigned:
                other = assigned[normalized]
                raise ValueError(
                    f"{sequence} 同时绑定了‘{SHORTCUT_LABELS[other]}’和"
                    f"‘{SHORTCUT_LABELS[action_id]}’"
                )
            assigned[normalized] = action_id
        return values

    def save_keyboard_shortcuts(self, _checked=False):
        try:
            values = self._collect_shortcut_values()
            self.teleop_store.update_runtime({"shortcuts": values})
            self._apply_keyboard_shortcuts()
            self.shortcut_status_label.setText("已保存并立即生效")
            self.shortcut_status_label.setStyleSheet("color: #17823b;")
            self._teleop_event("info", "键盘快捷键已保存并立即生效")
        except Exception as exc:
            self.shortcut_status_label.setText(f"保存失败：{exc}")
            self.shortcut_status_label.setStyleSheet("color: #c62828;")
            self._teleop_event("error", f"保存快捷键失败: {exc}")

    def _shortcut_action_handlers(self):
        # Keep the existing config keys for F9/F10, but deliberately remove
        # auto-restart and all hidden legacy motion shortcuts.
        return {
            "prepare": self.WorkflowPrepare,
            "power_on": self.WorkflowPowerOn,
            "start_follow": self.WorkflowStartTeleop,
            "stop_follow": self.TeleopStopCurrentAction,
            "episode_start": self.LeRobotEpisodeStart,
            "episode_save_next": self.LeRobotEpisodeSave,
            "episode_discard_retry": self.LeRobotEpisodeDiscard,
            "home": self.TeleopGoHome,
            "emergency_stop": self.EmergencyStop,
        }

    def _trigger_keyboard_action(self, action_id):
        focused = QApplication.focusWidget()
        if action_id != "emergency_stop" and isinstance(
            focused, (QLineEdit, QTextEdit, QPlainTextEdit, QAbstractSpinBox, QComboBox, QKeySequenceEdit)
        ):
            return
        guarded_widgets = {
            "prepare": self.workflow_prepare_button,
            "power_on": self.gello_page_power_on,
            "start_follow": self.gello_page_start,
            "stop_follow": self.gello_page_stop,
            "episode_start": self.lerobot_start_button,
            "episode_save_next": self.lerobot_save_button,
            "episode_discard_retry": self.lerobot_discard_button,
            "home": self.gello_page_go_home,
            "emergency_stop": self.gello_page_estop,
        }
        if not guarded_widgets[action_id].isEnabled():
            self._teleop_event("warning", f"当前状态不允许：{SHORTCUT_LABELS[action_id]}")
            return
        self._shortcut_action_handlers()[action_id]()

    def _apply_keyboard_shortcuts(self):
        for shortcut in self._keyboard_shortcuts.values():
            shortcut.setEnabled(False)
            shortcut.deleteLater()
        self._keyboard_shortcuts = {}
        configured = self.teleop_store.data["shortcuts"]
        for action_id in self._shortcut_action_handlers():
            sequence = str(configured[action_id]).strip()
            if sequence:
                shortcut = QShortcut(QKeySequence(sequence), self)
                shortcut.setContext(Qt.ApplicationShortcut)
                shortcut.activated.connect(lambda key=action_id: self._trigger_keyboard_action(key))
                self._keyboard_shortcuts[action_id] = shortcut
        for button, label, action_id in (
            (self.lerobot_start_button, "开始 Episode", "episode_start"),
            (self.lerobot_save_button, "保存当前", "episode_save_next"),
            (self.lerobot_discard_button, "丢弃当前", "episode_discard_retry"),
            (self.gello_page_stop, "停止运动并保持", "stop_follow"),
            (self.gello_page_estop, "软件紧急停止", "emergency_stop"),
        ):
            sequence = str(configured[action_id]).strip()
            button.setText(label + (f" [{sequence}]" if sequence else ""))

    @staticmethod
    def _set_rgb_pixmap(label, image_rgb):
        image = np.ascontiguousarray(image_rgb)
        height, width, channels = image.shape
        qt_image = QImage(
            image.data,
            width,
            height,
            channels * width,
            QImage.Format_RGB888,
        ).copy()
        pixmap = QPixmap.fromImage(qt_image)
        target = label.size()
        if target.width() > 0 and target.height() > 0:
            pixmap = pixmap.scaled(target, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        label.setPixmap(pixmap)

    def CamerasStartAll(self, _checked=False):
        self.D435_1_Start()
        self.D435_2_Start()

    def CamerasStopAll(self, _checked=False):
        dataset_snapshot = self.lerobot_recorder.snapshot()
        if dataset_snapshot["episode_active"]:
            self._teleop_event(
                "warning",
                "Episode 正在录制，已拒绝停止相机；请先保存或丢弃当前 Episode",
            )
            return
        self.D435_1_Stop()
        self.D435_2_Stop()

    def D435_1_Start(self):
        if self.D435_1_Started:
            self._teleop_event("info", "CAMERA1 腕部相机已经启动")
            return
        try:
            self.wrist_camera_device.connect(timeout=5.0)
            self.D435_1_Started = True
            timer = getattr(self, "timer_D435_1", None)
            if timer is None:
                self.timer_D435_1 = QTimer(self)
                self.timer_D435_1.timeout.connect(self.update_frame_D435_1)
            self.timer_D435_1.start(30)
            self._teleop_event(
                "info",
                f"CAMERA1 腕部相机已启动: {self.wrist_camera_serial}",
            )
        except Exception as exc:
            self.D435_1_Started = False
            self.wrist_camera_device.close()
            self._teleop_event("error", f"CAMERA1 腕部相机启动失败: {exc}")

    def D435_1_Stop(self):
        if (
            self.lerobot_recorder.snapshot()["episode_active"]
            or self._episode_operation_lock.locked()
        ):
            self._teleop_event(
                "warning",
                "请先结束录制并等待 Episode 操作完成，再停止相机",
            )
            return
        timer = getattr(self, "timer_D435_1", None)
        if timer is not None:
            timer.stop()
        try:
            self.wrist_camera_device.close()
        except Exception as exc:
            self._teleop_event("error", f"CAMERA1 停止失败: {exc}")
        self.D435_1_Started = False
        with self._camera_frame_lock:
            self._wrist_rgb_frame = None
            self._wrist_rgb_timestamp = 0.0
        self.D435_Pic_Color.clear()
        self.D435_Pic_Color.setText("腕部 RGB 已停止")
        self._teleop_event("info", "CAMERA1 腕部相机已停止")

    def D435_1_Cut(self):
        with self._camera_frame_lock:
            wrist = (
                None
                if self._wrist_rgb_frame is None
                else self._wrist_rgb_frame.copy()
            )
        if wrist is None:
            self._teleop_event("error", "CAMERA1 尚无腕部 RGB，无法截图")
            return
        output_dir = BASE_DIR / "data" / "camera1_wrist"
        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / f"wrist_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
        cv2.imwrite(str(path), cv2.cvtColor(wrist, cv2.COLOR_RGB2BGR))
        self._teleop_event("info", f"CAMERA1 腕部截图已保存: {path}")


    def update_frame_D435_1(self):
        if not self.D435_1_Started:
            return

        error = self.wrist_camera_device.error
        if error:
            timer = getattr(self, "timer_D435_1", None)
            if timer is not None:
                timer.stop()
            self.D435_1_Started = False
            self.wrist_camera_device.close()
            self._teleop_event(
                "error",
                f"腕部相机取帧失败，采集将停止：{error}",
            )
            return

        snapshot = self.wrist_camera_device.latest()
        if snapshot is None:
            return

        image_rgb = snapshot.image_rgb
        with self._camera_frame_lock:
            self._wrist_rgb_frame = image_rgb.copy()
            self._wrist_rgb_timestamp = float(snapshot.timestamp)
        self._set_rgb_pixmap(self.D435_Pic_Color, image_rgb)

    def D435_2_Start(self):
        if self.D435_2_Started:
            self._teleop_event("info", "CAMERA2 基座相机已经启动")
            return
        try:
            self.base_camera_device.connect(timeout=5.0)
            self.D435_2_Started = True
            timer = getattr(self, "timer_D435_2", None)
            if timer is None:
                self.timer_D435_2 = QTimer(self)
                self.timer_D435_2.timeout.connect(self.update_frame_D435_2)
            self.timer_D435_2.start(30)
            self._teleop_event(
                "info",
                f"CAMERA2 基座相机已启动: {self.base_camera_serial}; "
                f"ROI={list(self.base_roi_norm)}",
            )
        except Exception as exc:
            self.D435_2_Started = False
            self.base_camera_device.close()
            self._teleop_event("error", f"CAMERA2 基座相机启动失败: {exc}")

    def D435_2_Stop(self):
        if (
            self.lerobot_recorder.snapshot()["episode_active"]
            or self._episode_operation_lock.locked()
        ):
            self._teleop_event(
                "warning",
                "请先结束录制并等待 Episode 操作完成，再停止相机",
            )
            return
        timer = getattr(self, "timer_D435_2", None)
        if timer is not None:
            timer.stop()
        try:
            self.base_camera_device.close()
        except Exception as exc:
            self._teleop_event("error", f"CAMERA2 停止失败: {exc}")
        self.D435_2_Started = False
        with self._camera_frame_lock:
            self._base_rgb_frame = None
            self._base_roi_rgb_frame = None
            self._base_rgb_timestamp = 0.0
        self.D435_2_Pic_Color.clear()
        self.D435_2_Pic_Color.setText("基座 RGB 已停止")
        self.D435_2_Pic_ROI.clear()
        self.D435_2_Pic_ROI.setText("基座 ROI 已停止")
        self._teleop_event("info", "CAMERA2 基座相机已停止")

    def D435_2_Cut(self):
        with self._camera_frame_lock:
            base = None if self._base_rgb_frame is None else self._base_rgb_frame.copy()
            roi = (
                None
                if self._base_roi_rgb_frame is None
                else self._base_roi_rgb_frame.copy()
            )
        if base is None or roi is None:
            self._teleop_event("error", "CAMERA2 尚无基座/ROI 图像，无法截图")
            return
        output_dir = BASE_DIR / "data" / "camera2_roi"
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base_path = output_dir / f"base_{stamp}.jpg"
        roi_path = output_dir / f"roi_{stamp}.jpg"
        cv2.imwrite(str(base_path), cv2.cvtColor(base, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(roi_path), cv2.cvtColor(roi, cv2.COLOR_RGB2BGR))
        self._teleop_event("info", f"CAMERA2 截图已保存: {base_path}; {roi_path}")

    def update_frame_D435_2(self):
        if not self.D435_2_Started:
            return

        error = self.base_camera_device.error
        if error:
            timer = getattr(self, "timer_D435_2", None)
            if timer is not None:
                timer.stop()
            self.D435_2_Started = False
            self.base_camera_device.close()
            self._teleop_event(
                "error",
                f"基座相机取帧失败，采集将停止：{error}",
            )
            return

        snapshot = self.base_camera_device.latest()
        if snapshot is None:
            return

        base_rgb = snapshot.image_rgb
        roi_crop = crop_normalized_roi(base_rgb, self.base_roi_norm)
        left, top, right, bottom = roi_crop.bounds_px
        roi_rgb = roi_crop.image_rgb

        with self._camera_frame_lock:
            self._base_rgb_frame = base_rgb.copy()
            self._base_roi_rgb_frame = roi_rgb
            self._base_rgb_timestamp = float(snapshot.timestamp)

        overlay = base_rgb.copy()
        cv2.rectangle(
            overlay,
            (left, top),
            (right - 1, bottom - 1),
            (255, 0, 0),
            2,
        )
        self._set_rgb_pixmap(self.D435_2_Pic_Color, overlay)
        self._set_rgb_pixmap(self.D435_2_Pic_ROI, roi_rgb)
        self.D435_2_Status.setText(
            f"基座序列号 {self.base_camera_serial} | "
            f"ROI {list(self.base_roi_norm)} | "
            f"像素 ({left},{top})→({right},{bottom}) | "
            f"裁剪 {right-left}×{bottom-top}"
        )

    def _init_linker_hand_type(self):
        try:
            self.yaml = LoadWriteYaml() # 初始化配置文件
            # 读取配置文件
            self.setting = self.yaml.load_setting_yaml()
            time.sleep(1)
            self.left_hand = False
            self.right_hand = False

            if self.setting['LINKER_HAND']['LEFT_HAND']['EXISTS'] == True:
                self.left_hand = True
            elif self.setting['LINKER_HAND']['RIGHT_HAND']['EXISTS'] == True:
                self.right_hand = True
            # gui控制只支持单手，这里进行左右手互斥
            if self.left_hand == True and self.right_hand == True:
                self.left_hand = True
                self.right_hand = False
            if self.left_hand == True:
                self.hand_exists = True
                self.hand_joint = self.setting['LINKER_HAND']['LEFT_HAND']['JOINT']
                self.hand_type = "left"
                self.is_touch = self.setting['LINKER_HAND']['LEFT_HAND']['TOUCH']
                self.can = self.setting['LINKER_HAND']['LEFT_HAND']['CAN']
                self.modbus = self.setting['LINKER_HAND']['LEFT_HAND']['MODBUS']
            if self.right_hand == True:
                self.hand_exists = True
                self.hand_joint = self.setting['LINKER_HAND']['RIGHT_HAND']['JOINT']
                self.hand_type = "right"
                self.is_touch = self.setting['LINKER_HAND']['RIGHT_HAND']['TOUCH']
                self.can = self.setting['LINKER_HAND']['RIGHT_HAND']['CAN']
                self.modbus = self.setting['LINKER_HAND']['RIGHT_HAND']['MODBUS']
        except Exception as e:
            ColorMsg(msg=f"Error 配置文件读取失败: {str(e)}", color="red")
            #self.status_updated.emit("error", f"配置文件读取失败: {str(e)}")
        ColorMsg(msg=f"当前配置为:Linker Hand {self.hand_type} {self.hand_joint} 压感:{self.is_touch} modbus:{self.modbus} CAN:{self.can}", color="green")

    def init_linker_hand_api(self):
        """初始化LinkerHandApi"""
        try:
            self.api = LinkerHandApi(hand_joint=self.hand_joint, hand_type=self.hand_type, modbus=self.modbus, can=self.can)
            #self.status_updated.emit("info", f"手部API初始化成功: {self.hand_type} {self.hand_joint}")
        except Exception as e:
            #self.status_updated.emit("error", f"API初始化失败: {str(e)}")
            raise

    def Gripper1_INIT(self):
        self.RoArmDevicesConnect()

    def Gripper1_Open(self):
        if self.teleop_engine.state in ("replay", "episode_replay"):
            self._teleop_event("warning", "轨迹重放期间 O6 由轨迹控制，已拒绝张开命令")
            return
        if not self.o6_controller.connected:
            self._teleop_event("error", "O6 未连接")
            return
        self.o6_controller.set_target(self.teleop_store.data["o6"]["open"])

    def Gripper1_Close(self):
        if self.teleop_engine.state in ("replay", "episode_replay"):
            self._teleop_event("warning", "轨迹重放期间 O6 由轨迹控制，已拒绝闭合命令")
            return
        if not self.o6_controller.connected:
            self._teleop_event("error", "O6 未连接")
            return
        self.o6_controller.set_target(self.teleop_store.data["o6"]["closed"])

    def Gripper1_GetPos(self):
        position, fault, _ = self.o6_controller.latest()
        if position is None:
            self._teleop_event("error", "O6 尚无位置反馈")
            return
        global Hand_1_Pos
        global Hand_1_Error
        Hand_1_Pos[:] = list(position)
        Hand_1_Error[:] = list(fault or [0] * 6)
        fields = (
            self.txtGripper1_TargetJ0,
            self.txtGripper1_TargetJ1,
            self.txtGripper1_TargetJ2,
            self.txtGripper1_TargetJ3,
            self.txtGripper1_TargetJ4,
            self.txtGripper1_TargetJ5,
        )
        for field, value in zip(fields, position):
            field.setPlainText(str(value))
        self.on_timeout_render_hand()

    def Gripper1_SetPos(self):
        if self.teleop_engine.state in ("replay", "episode_replay"):
            self._teleop_event("warning", "轨迹重放期间 O6 由轨迹控制，已拒绝位置命令")
            return
        try:
            target_fields = [getattr(self, f"txtGripper1_TargetJ{i}") for i in range(6)]
            speed_fields = [getattr(self, f"txtGripper1_TargetVel{i}") for i in range(6)]
            torque_fields = [getattr(self, f"txtGripper1_TargetTorque{i}") for i in range(6)]
            target = [int(field.toPlainText()) for field in target_fields]
            speed = [int(field.toPlainText()) for field in speed_fields]
            torque = [int(field.toPlainText()) for field in torque_fields]
            self.o6_controller.set_profile(speed, torque)
            self.o6_controller.set_target(target)
        except Exception as exc:
            self._teleop_event("error", f"O6 目标参数无效: {exc}")

    # WEIXUE ARM.............................................................
    def WEIXUE1_INIT(self):
        #port = self.textEdit.toPlainText();
        print("WEIXUE1_INIT PORT=")

        # start hand control
        self.timer_render_weixue = QTimer(self)
        self.timer_render_weixue.timeout.connect(self.on_timeout_render_weixue)
        self.timer_render_weixue.start(30)

    def WEIXUE_HOME_ALL(self):
        self._teleop_event("warning", "WEIXUE 回零功能尚未接入，本次未下发命令")

    def WEIXUE1_FollowStart(self):
        self._teleop_event("warning", "WEIXUE 跟随功能尚未接入")

    def WEIXUE1_FollowStop(self):
        self._teleop_event("info", "WEIXUE 跟随已停止")

    def update_WEIXUE_1(self):
        # 保留旧页面；未连接 WEIXUE 硬件时不进行 I/O。
        return


    # INEXBOT ROBOT.............................................................
    def RobotCONNECT(self):
        if self.robot1_connected and self.nrc_adapter is not None:
            self._teleop_event("info", "纳博特控制柜已连接，未重复打开端口")
            return
        if not self.save_teleop_settings(show_event=False):
            return
        if not hasattr(self, "timer_render"):
            self.timer_render = QTimer(self)
            self.timer_render.timeout.connect(self.on_timeout_render)
            self.timer_render.start(200)

        def connect_controller():
            cfg = self.teleop_store.data["robot"]
            device = None

            def forward_cr3a_event(level, message):
                message_text = str(message)
                message_key = (str(level), message_text)
                now = time.monotonic()
                if (
                    str(level) == "warning"
                    and now - self._nrc_message_last.get(message_key, 0.0) < 1.0
                ):
                    return
                self._nrc_message_last[message_key] = now
                self._teleop_event(str(level), message_text)

            try:
                old_device = self.cr3a_device
                if old_device is not None:
                    old_device.close()

                self.cr3a_device = None
                self.nrc_adapter = None
                self.socketFd = -1
                self.socketFd_7000 = -1

                device = Cr3aDevice(
                    Cr3aConfig(
                        ip=str(cfg["ip"]),
                        command_port=int(cfg["command_port"]),
                        servo_port=int(cfg["servo_port"]),
                        robot_num=int(cfg["robot_num"]),
                        motion_mode=str(cfg["motion_mode"]),
                        movej_velocity=float(cfg["movej_velocity"]),
                        movej_acc=float(cfg["movej_acc"]),
                        movej_dec=float(cfg["movej_dec"]),
                        movej_period_s=float(cfg["movej_period_s"]),
                        movej_low_latency=bool(cfg["movej_low_latency"]),
                        servoj_vmax=float(cfg["servoj_vmax"]),
                        servoj_amax=float(cfg["servoj_amax"]),
                        servoj_jmax=float(cfg["servoj_jmax"]),
                        sdk_root=str(BASE_DIR / "TESTRobot_INEXBOT"),
                    ),
                    event_callback=forward_cr3a_event,
                )

                device.connect(timeout=10.0)
                session = device.session
                if session is None:
                    raise RuntimeError("CR3A 已连接但 NRC session 未创建")

                self.cr3a_device = device
                self.socketFd = device.command_fd
                self.socketFd_7000 = device.servo_fd
                self.nrc_adapter = session

                self.teleop_engine.attach_robot(session)
                self.robot1_connected = True
                self._teleop_event(
                    "info",
                    f"纳博特控制柜已连接: {cfg['ip']}:{cfg['command_port']}/"
                    f"{cfg['servo_port']}，robotNum={cfg['robot_num']}",
                )
            except Exception:
                self.robot1_connected = False
                self.nrc_adapter = None
                self.socketFd = -1
                self.socketFd_7000 = -1
                self.cr3a_device = None
                if device is not None:
                    device.close()
                raise

        self._run_teleop_async("连接纳博特控制柜", connect_controller)

        # # 开始运动队列（队列很垃圾，不要用）
        # result1 = aa.queue_motion_set_status(self.socketFd, True)
        # if result1 == 0:
        #     print('start queue_motion OK')
        # else:
        #     print('start queue_motion NG')

    # movej command def queue_cmd(socketFd, name, window):
    def queue_cmd_movej(self,target_joint_pos):
        global Size
        global pos

        # 创建并填充 VectorDouble
        pos = aa.VectorDouble()
        for value in target_joint_pos:
            pos.append(value)  # 使用 append 方法逐个添加值

        cmd = aa.MoveCmd()
        cmd.targetPosType = 0  # aa.PosType_data
        cmd.targetPosValue = pos
        print('cmd targetPosType:', cmd.targetPosType)  # movej=0
        print('cmd:', list(cmd.targetPosValue))

        cmd.velocity = 80
        cmd.acc = 100
        cmd.dec = 100

        result2 = aa.queue_motion_push_back_moveJ(self.socketFd, cmd)
        if result2 == 0:
            print('start movej OK')
        else:
            print('start movej NG')
        # 更新窗口的 QTextEdit 文本
        # window.update_edit_text("movJ")

    # movel command
    def queue_cmd_movel(self, target_tcp_pos):
        global Size
        global pos

        # 创建并填充 VectorDouble
        pos = aa.VectorDouble()
        for value in target_tcp_pos:
            pos.append(value)  # 使用 append 方法逐个添加值

        cmd = aa.MoveCmd()
        cmd.targetPosType = 1  # aa.PosType_data
        cmd.targetPosValue = pos
        print('cmd targetPosType:', cmd.targetPosType)  # movej=0
        print('cmd:', list(cmd.targetPosValue))

        cmd.velocity = 80
        cmd.acc = 100
        cmd.dec = 100

        result = aa.queue_motion_push_back_moveL(self.socketFd, cmd)
        if result == 0:
            # Size += 1
            print('movel executed')
        else:
            print('movel failed')

    def RobotPowerON(self):
        self.WorkflowPowerOn()

    def RobotPowerOFF(self):
        def power_off():
            if self.nrc_adapter is None:
                raise RuntimeError("CR5 未连接")
            self.teleop_engine.emergency_stop("下使能前停止遥操")
            transition = self.nrc_adapter.power_off()
            self._teleop_event(
                "info",
                f"CR5 已下使能（{transition.initial_state}→{transition.final_state}）",
            )

        self._run_teleop_async("CR5 下使能", power_off)

    def RobotClearError(self):
        self.WorkflowPowerOn()

    def RobotTrackStart(self):
        self.TeleopFollowStart()

    def RobotTrackStop(self):
        self.TeleopStopCurrentAction()

    def RobotGoHome(self):
        print("RobotGoHome")
        self.txtJointPos0.setPlainText("-90")
        self.txtJointPos1.setPlainText("10")
        self.txtJointPos2.setPlainText("-15")
        self.txtJointPos3.setPlainText("-2")
        self.txtJointPos4.setPlainText("-2")
        self.txtJointPos5.setPlainText("0")

    def RobotGoFlat(self):
        print("RobotGoFlat")
        self.txtTCPPosA.setPlainText("-3.14")
        self.txtTCPPosB.setPlainText("0")
        self.txtTCPPosC.setPlainText("0")

    def RobotGetJ(self):
        print("RobotGetJ")

        # get current joint position
        current_pos = aa.VectorDouble()
        coord = 0
        aa.get_current_position(self.socketFd, coord, current_pos)
        pos_list = list(current_pos)

        if len(pos_list) > 0:
           self.txtJointPos0.setPlainText(str(round(pos_list[0], 2)))
           self.txtJointPos1.setPlainText(str(round(pos_list[1], 2)))
           self.txtJointPos2.setPlainText(str(round(pos_list[2], 2)))
           self.txtJointPos3.setPlainText(str(round(pos_list[3], 2)))
           self.txtJointPos4.setPlainText(str(round(pos_list[4], 2)))
           self.txtJointPos5.setPlainText(str(round(pos_list[5], 2)))
           self.txtJointPos6.setPlainText(str(round(pos_list[6], 2)))

        else:
            print("get robot joint pos error")

    def RobotMoveJ(self):
        if self.nrc_adapter is None:
            self._teleop_event("error", "CR5 未连接")
            return
        try:
            target_joint = [
                float(getattr(self, f"txtJointPos{i}").toPlainText() or "0")
                for i in range(7)
            ]
        except ValueError as exc:
            self._teleop_event("error", f"MoveJ 目标无效: {exc}")
            return
        preset = self.teleop_store.data["preset"]
        speed = float(preset["robot_velocity_percent"])
        acc = float(preset["robot_acc_percent"])
        dec = float(preset["robot_dec_percent"])
        self._run_teleop_async(
            "CR5 MoveJ",
            lambda: self.nrc_adapter.movej(target_joint, speed, acc, dec),
        )

    def RobotGetL(self):
        print("RobotGetL")
        # get current tcp position
        current_pos = aa.VectorDouble()
        coord = 1
        aa.get_current_position(self.socketFd, coord, current_pos)
        pos_list = list(current_pos)

        if len(pos_list) > 0:
           self.txtTCPPosX.setPlainText(str(round(pos_list[0], 2)))
           self.txtTCPPosY.setPlainText(str(round(pos_list[1], 2)))
           self.txtTCPPosZ.setPlainText(str(round(pos_list[2], 2)))
           self.txtTCPPosA.setPlainText(str(round(pos_list[3], 2)))
           self.txtTCPPosB.setPlainText(str(round(pos_list[4], 2)))
           self.txtTCPPosC.setPlainText(str(round(pos_list[5], 2)))
           self.txtTCPPosEXT.setPlainText(str(round(pos_list[6], 2)))
        else:
            print("get robot tcp pos error")

    def RobotMoveL(self):
        if self.nrc_adapter is None:
            self._teleop_event("error", "CR5 未连接")
            return
        try:
            fields = (
                self.txtTCPPosX,
                self.txtTCPPosY,
                self.txtTCPPosZ,
                self.txtTCPPosA,
                self.txtTCPPosB,
                self.txtTCPPosC,
                self.txtTCPPosEXT,
            )
            target_tcp = [float(field.toPlainText() or "0") for field in fields]
        except ValueError as exc:
            self._teleop_event("error", f"MoveL 目标无效: {exc}")
            return
        preset = self.teleop_store.data["preset"]
        speed = float(preset["robot_velocity_percent"])
        acc = float(preset["robot_acc_percent"])
        dec = float(preset["robot_dec_percent"])
        self._run_teleop_async(
            "CR5 MoveL",
            lambda: self.nrc_adapter.movel(target_tcp, speed, acc, dec),
        )

    def RobotGetKernalParameter(self):
        param = aa.RobotJointParam()
        status = aa.get_robot_joint_param(self.socketFd, 1, param)
        print('-------', param.reducRatio)
        dhparam = aa.RobotDHParam()
        status = aa.get_robot_dh_param(self.socketFd, dhparam)
        print('-------', dhparam.L1)

    def Track_ActionJ0(self):
        value = self.horizontalScrollBarTestValueJ0.value()  # / 10.0 # -100 to 100
        self.label_J0.setText(str(value))

        target_joint= [0,0,0,0,0,0,0]
        for i in range(7):
            target_joint[i] = self.robot1_joint_pos[i]

        # only change one joint pos
        target_joint[0] = self.robot1_joint_pos[0] + value

        testrobotmessage = "target_joint=" + str(round(target_joint[0], 2)) + "," + str(round(target_joint[1], 2)) + "," + str(round(target_joint[2], 2)) + "," + str(round(target_joint[3], 2)) + "," + str(round(target_joint[4], 2)) + "," + str(round(target_joint[5], 2)) + "," + str(round(target_joint[6], 2)) + "\n"
        print(testrobotmessage)
        aa.set_servoJ_pos(self.socketFd_7000, target_joint)

        # # change to [-1,1]
        # value = self.horizontalScrollBarTestValueJ0.value()  / 180.0
        # self.label_J0.setText(str(value))
        # aa.set_current_coord(self.socketFd_7000,0) #coord	坐标系 0：关节 1：直角 2：工具 3：用户
        #
        # if abs(value)>0.1:
        #     aa.robot_start_jogging(self.socketFd_7000, 1, value)
        # else :
        #     aa.robot_stop_jogging(self.socketFd_7000,1)

    def Track_ActionJ1(self):
        value = self.horizontalScrollBarTestValueJ1.value()  # / 10.0 # -100 to 100
        self.label_J1.setText(str(value))

        target_joint= [0,0,0,0,0,0,0]
        for i in range(7):
            target_joint[i] = self.robot1_joint_pos[i]

        # only change one joint pos
        target_joint[1] = self.robot1_joint_pos[1] + value

        testrobotmessage = "target_joint=" + str(round(target_joint[0], 2)) + "," + str(round(target_joint[1], 2)) + "," + str(round(target_joint[2], 2)) + "," + str(round(target_joint[3], 2)) + "," + str(round(target_joint[4], 2)) + "," + str(round(target_joint[5], 2)) + "," + str(round(target_joint[6], 2)) + "\n"
        print(testrobotmessage)
        aa.set_servoJ_pos(self.socketFd_7000, target_joint)

    def Track_ActionJ2(self):
        value = self.horizontalScrollBarTestValueJ2.value()  # / 10.0 # -100 to 100
        self.label_J2.setText(str(value))

        target_joint= [0,0,0,0,0,0,0]
        for i in range(7):
            target_joint[i] = self.robot1_joint_pos[i]

        # only change one joint pos
        target_joint[2] = self.robot1_joint_pos[2] + value

        testrobotmessage = "target_joint=" + str(round(target_joint[0], 2)) + "," + str(round(target_joint[1], 2)) + "," + str(round(target_joint[2], 2)) + "," + str(round(target_joint[3], 2)) + "," + str(round(target_joint[4], 2)) + "," + str(round(target_joint[5], 2)) + "," + str(round(target_joint[6], 2)) + "\n"
        print(testrobotmessage)
        aa.set_servoJ_pos(self.socketFd_7000, target_joint)

    def Track_ActionJ3(self):
        value = self.horizontalScrollBarTestValueJ3.value()  # / 10.0 # -100 to 100
        self.label_J3.setText(str(value))

        target_joint= [0,0,0,0,0,0,0]
        for i in range(7):
            target_joint[i] = self.robot1_joint_pos[i]

        # only change one joint pos
        target_joint[3] = self.robot1_joint_pos[3] + value

        testrobotmessage = "target_joint=" + str(round(target_joint[0], 2)) + "," + str(round(target_joint[1], 2)) + "," + str(round(target_joint[2], 2)) + "," + str(round(target_joint[3], 2)) + "," + str(round(target_joint[4], 2)) + "," + str(round(target_joint[5], 2)) + "," + str(round(target_joint[6], 2)) + "\n"
        print(testrobotmessage)
        aa.set_servoJ_pos(self.socketFd_7000, target_joint)

    def Track_ActionJ4(self):
        value = self.horizontalScrollBarTestValueJ4.value()  # / 10.0 # -100 to 100
        self.label_J4.setText(str(value))

        target_joint= [0,0,0,0,0,0,0]
        for i in range(7):
            target_joint[i] = self.robot1_joint_pos[i]

        # only change one joint pos
        target_joint[4] = self.robot1_joint_pos[4] + value

        testrobotmessage = "target_joint=" + str(round(target_joint[0], 2)) + "," + str(round(target_joint[1], 2)) + "," + str(round(target_joint[2], 2)) + "," + str(round(target_joint[3], 2)) + "," + str(round(target_joint[4], 2)) + "," + str(round(target_joint[5], 2)) + "," + str(round(target_joint[6], 2)) + "\n"
        print(testrobotmessage)
        aa.set_servoJ_pos(self.socketFd_7000, target_joint)

    def Track_ActionJ5(self):
        value = self.horizontalScrollBarTestValueJ5.value()  # / 10.0 # -100 to 100
        self.label_J5.setText(str(value))

        target_joint= [0,0,0,0,0,0,0]
        for i in range(7):
            target_joint[i] = self.robot1_joint_pos[i]

        # only change one joint pos
        target_joint[5] = self.robot1_joint_pos[5] + value

        testrobotmessage = "target_joint=" + str(round(target_joint[0], 2)) + "," + str(round(target_joint[1], 2)) + "," + str(round(target_joint[2], 2)) + "," + str(round(target_joint[3], 2)) + "," + str(round(target_joint[4], 2)) + "," + str(round(target_joint[5], 2)) + "," + str(round(target_joint[6], 2)) + "\n"
        print(testrobotmessage)
        aa.set_servoJ_pos(self.socketFd_7000, target_joint)

    def Track_ActionJ6(self):
        value = self.horizontalScrollBarTestValueJ6.value()  # / 10.0 # -100 to 100
        self.label_J6.setText(str(value))

        target_joint= [0,0,0,0,0,0,0]
        for i in range(7):
            target_joint[i] = self.robot1_joint_pos[i]

        # only change one joint pos
        target_joint[6] = self.robot1_joint_pos[6] + value

        testrobotmessage = "target_joint=" + str(round(target_joint[0], 2)) + "," + str(round(target_joint[1], 2)) + "," + str(round(target_joint[2], 2)) + "," + str(round(target_joint[3], 2)) + "," + str(round(target_joint[4], 2)) + "," + str(round(target_joint[5], 2)) + "," + str(round(target_joint[6], 2)) + "\n"
        print(testrobotmessage)
        aa.set_servoJ_pos(self.socketFd_7000, target_joint)

    def Track_ActionX(self):
        value = self.horizontalScrollBarTestValueX.value()  # / 10.0 # -100 to 100
        self.label_X.setText(str(value))

        target_tcp= [0,0,0,0,0,0,0]
        for i in range(7):
            target_tcp[i] = self.robot1_tcp_pos[i]

        # only change one tcp pos
        target_tcp[0] = self.robot1_tcp_pos[0] + value

        testrobotmessage = "target_tcp=" + str(round(target_tcp[0], 2)) + "," + str(
            round(target_tcp[1], 2)) + "," + str(round(target_tcp[2], 2)) + "," + str(
            round(target_tcp[3], 2)) + "," + str(round(target_tcp[4], 2)) + "," + str(
            round(target_tcp[5], 2)) + "," + str(round(target_tcp[6], 2)) + "\n"
        print(testrobotmessage)

        t = aa.VectorDouble(7)
        # if can get inverse kin to t,print t!!!
        if (aa.get_origin_coord_to_target_coord(self.socketFd, 1, target_tcp, 0, t) == 0):
            target_joint_str = str(t[0]) + "," + str(t[1]) + "," + str(t[2]) + "," + str(t[3]) + "," + str(t[4]) + "," + str(t[5]) + "," + str(t[6])
            print('target_joint_str t=', target_joint_str)
            aa.set_servoJ_pos(self.socketFd_7000, t)

    def Track_ActionY(self):
        value = self.horizontalScrollBarTestValueY.value()  # / 10.0 # -100 to 100
        self.label_Y.setText(str(value))

        target_tcp = [0, 0, 0, 0, 0, 0, 0]
        for i in range(7):
            target_tcp[i] = self.robot1_tcp_pos[i]

        # only change one tcp pos
        target_tcp[1] = self.robot1_tcp_pos[1] + value

        testrobotmessage = "target_tcp=" + str(round(target_tcp[0], 2)) + "," + str(
            round(target_tcp[1], 2)) + "," + str(round(target_tcp[2], 2)) + "," + str(
            round(target_tcp[3], 2)) + "," + str(round(target_tcp[4], 2)) + "," + str(
            round(target_tcp[5], 2)) + "," + str(round(target_tcp[6], 2)) + "\n"
        print(testrobotmessage)

        t = aa.VectorDouble(7)
        # if can get inverse kin to t,print t!!!
        if (aa.get_origin_coord_to_target_coord(self.socketFd, 1, target_tcp, 0, t) == 0):
            target_joint_str = str(t[0]) + "," + str(t[1]) + "," + str(t[2]) + "," + str(t[3]) + "," + str(
                t[4]) + "," + str(t[5]) + "," + str(t[6])
            print('target_joint_str t=', target_joint_str)
            aa.set_servoJ_pos(self.socketFd_7000, t)

    def Track_ActionZ(self):
        value = self.horizontalScrollBarTestValueZ.value()  # / 10.0 # -100 to 100
        self.label_Z.setText(str(value))

        target_tcp = [0, 0, 0, 0, 0, 0, 0]
        for i in range(7):
            target_tcp[i] = self.robot1_tcp_pos[i]

        # only change one tcp pos
        target_tcp[2] = self.robot1_tcp_pos[2] + value

        testrobotmessage = "target_tcp=" + str(round(target_tcp[0], 2)) + "," + str(
            round(target_tcp[1], 2)) + "," + str(round(target_tcp[2], 2)) + "," + str(
            round(target_tcp[3], 2)) + "," + str(round(target_tcp[4], 2)) + "," + str(
            round(target_tcp[5], 2)) + "," + str(round(target_tcp[6], 2)) + "\n"
        print(testrobotmessage)

        t = aa.VectorDouble(7)
        # if can get inverse kin to t,print t!!!
        if (aa.get_origin_coord_to_target_coord(self.socketFd, 1, target_tcp, 0, t) == 0):
            target_joint_str = str(t[0]) + "," + str(t[1]) + "," + str(t[2]) + "," + str(t[3]) + "," + str(
                t[4]) + "," + str(t[5]) + "," + str(t[6])
            print('target_joint_str t=', target_joint_str)
            aa.set_servoJ_pos(self.socketFd_7000, t)

    def Track_ActionA(self):
        # ius should
        value = math.radians(self.horizontalScrollBarTestValueA.value())  # / 10.0 # -100 to 100
        self.label_A.setText(str(value))

        target_tcp = [0, 0, 0, 0, 0, 0, 0]
        for i in range(7):
            target_tcp[i] = self.robot1_tcp_pos[i]

        # only change one tcp pos
        target_tcp[3] = self.robot1_tcp_pos[3] + value

        testrobotmessage = "target_tcp=" + str(round(target_tcp[0], 2)) + "," + str(
            round(target_tcp[1], 2)) + "," + str(round(target_tcp[2], 2)) + "," + str(
            round(target_tcp[3], 2)) + "," + str(round(target_tcp[4], 2)) + "," + str(
            round(target_tcp[5], 2)) + "," + str(round(target_tcp[6], 2)) + "\n"
        print(testrobotmessage)

        t = aa.VectorDouble(7)
        # if can get inverse kin to t,print t!!!
        if (aa.get_origin_coord_to_target_coord(self.socketFd, 1, target_tcp, 0, t) == 0):
            target_joint_str = str(t[0]) + "," + str(t[1]) + "," + str(t[2]) + "," + str(t[3]) + "," + str(
                t[4]) + "," + str(t[5]) + "," + str(t[6])
            print('target_joint_str t=', target_joint_str)
            aa.set_servoJ_pos(self.socketFd_7000, t)

    def Track_ActionB(self):
        value = math.radians(self.horizontalScrollBarTestValueB.value())  # / 10.0 # -100 to 100
        self.label_B.setText(str(value))

        target_tcp = [0, 0, 0, 0, 0, 0, 0]
        for i in range(7):
            target_tcp[i] = self.robot1_tcp_pos[i]

        # only change one tcp pos
        target_tcp[4] = self.robot1_tcp_pos[4] + value

        testrobotmessage = "target_tcp=" + str(round(target_tcp[0], 2)) + "," + str(
            round(target_tcp[1], 2)) + "," + str(round(target_tcp[2], 2)) + "," + str(
            round(target_tcp[3], 2)) + "," + str(round(target_tcp[4], 2)) + "," + str(
            round(target_tcp[5], 2)) + "," + str(round(target_tcp[6], 2)) + "\n"
        print(testrobotmessage)

        t = aa.VectorDouble(7)
        # if can get inverse kin to t,print t!!!
        if (aa.get_origin_coord_to_target_coord(self.socketFd, 1, target_tcp, 0, t) == 0):
            target_joint_str = str(t[0]) + "," + str(t[1]) + "," + str(t[2]) + "," + str(t[3]) + "," + str(
                t[4]) + "," + str(t[5]) + "," + str(t[6])
            print('target_joint_str t=', target_joint_str)
            aa.set_servoJ_pos(self.socketFd_7000, t)

    def Track_ActionC(self):
        value = math.radians(self.horizontalScrollBarTestValueC.value()) # / 10.0 # -100 to 100
        self.label_C.setText(str(value))

        target_tcp = [0, 0, 0, 0, 0, 0, 0]
        for i in range(7):
            target_tcp[i] = self.robot1_tcp_pos[i]

        # only change one tcp pos
        target_tcp[5] = self.robot1_tcp_pos[5] + value

        testrobotmessage = "target_tcp=" + str(round(target_tcp[0], 2)) + "," + str(
            round(target_tcp[1], 2)) + "," + str(round(target_tcp[2], 2)) + "," + str(
            round(target_tcp[3], 2)) + "," + str(round(target_tcp[4], 2)) + "," + str(
            round(target_tcp[5], 2)) + "," + str(round(target_tcp[6], 2)) + "\n"
        print(testrobotmessage)

        t = aa.VectorDouble(7)
        # if can get inverse kin to t,print t!!!
        if (aa.get_origin_coord_to_target_coord(self.socketFd, 1, target_tcp, 0, t) == 0):
            target_joint_str = str(t[0]) + "," + str(t[1]) + "," + str(t[2]) + "," + str(t[3]) + "," + str(
                t[4]) + "," + str(t[5]) + "," + str(t[6])
            print('target_joint_str t=', target_joint_str)
            aa.set_servoJ_pos(self.socketFd_7000, t)

    # 显示机器人数据
    def on_timeout_render(self):
        if not self.robot1_connected or self.nrc_adapter is None:
            return
        global ROBOT1_JointPOS
        global ROBOT1_TCPPOS
        now = time.monotonic()
        teleop_snapshot = self._latest_teleop_snapshot
        telemetry = (
            teleop_snapshot.get("gello_telemetry")
            if teleop_snapshot is not None else None
        )
        following = (
            teleop_snapshot is not None
            and teleop_snapshot.get("state") == "following"
            and telemetry is not None
            and now - telemetry["feedback_at"]
            <= float(self.teleop_store.data["gello"]["feedback_timeout_s"])
        )
        poll_interval = 0.75 if following else 0.2
        poll_thread = getattr(self, "_robot_poll_thread", None)
        if ((poll_thread is None or not poll_thread.is_alive())
                and now - self._last_robot_poll_started_at >= poll_interval):
            adapter = self.nrc_adapter
            follow_telemetry = telemetry if following else None
            previous_snapshot = getattr(self, "_latest_robot_snapshot", None)

            def poll_robot():
                try:
                    if follow_telemetry is None:
                        joints = adapter.joint_position()
                        tcp = adapter.tcp_position()
                    else:
                        actual = list(follow_telemetry["actual_deg"])
                        joint_7 = (
                            previous_snapshot[0][6]
                            if previous_snapshot is not None
                            and not previous_snapshot[3]
                            and previous_snapshot[0] is not None
                            and len(previous_snapshot[0]) > 6
                            else 0.0
                        )
                        joints = actual + [joint_7]
                        tcp = (
                            previous_snapshot[1]
                            if previous_snapshot is not None
                            and not previous_snapshot[3]
                            and previous_snapshot[1] is not None
                            else adapter.tcp_position()
                        )
                    self._latest_robot_snapshot = (joints, tcp, adapter.servo_state(), "")
                    self._latest_robot_snapshot_at = time.monotonic()
                except Exception as exc:
                    self._latest_robot_snapshot = (None, None, None, str(exc))
                    self._latest_robot_snapshot_at = 0.0

            self._last_robot_poll_started_at = now
            self._robot_poll_thread = threading.Thread(
                target=poll_robot, name="NRC-status-poll", daemon=True
            )
            self._robot_poll_thread.start()

        snapshot = getattr(self, "_latest_robot_snapshot", None)
        if snapshot is None:
            return
        joints, tcp, servo_state, poll_error = snapshot
        if poll_error:
            if poll_error != getattr(self, "_last_robot_poll_error", ""):
                self._last_robot_poll_error = poll_error
                self._teleop_event("error", f"读取 CR5 状态失败: {poll_error}")
            return
        try:
            self.robot1_joint_pos[:] = joints[:7]
            self.robot1_tcp_pos[:] = tcp[:7]
            ROBOT1_JointPOS[:] = joints[:7]
            ROBOT1_TCPPOS[:] = tcp[:7]

            for field, value in zip(
                (
                    self.txtRobot1J0,
                    self.txtRobot1J1,
                    self.txtRobot1J2,
                    self.txtRobot1J3,
                    self.txtRobot1J4,
                    self.txtRobot1J5,
                    self.txtRobot1J6,
                ),
                joints,
            ):
                field.setText(str(round(value, 2)))
            for field, value in zip(
                (
                    self.txtRobot1J0_WEIXUE,
                    self.txtRobot1J1_WEIXUE,
                    self.txtRobot1J2_WEIXUE,
                    self.txtRobot1J3_WEIXUE,
                    self.txtRobot1J4_WEIXUE,
                    self.txtRobot1J5_WEIXUE,
                    self.txtRobot1J6_WEIXUE,
                ),
                joints,
            ):
                field.setText(str(round(value, 2)))
            for field, value in zip(
                (
                    self.txtRobot1_X_WEIXUE,
                    self.txtRobot1_Y_WEIXUE,
                    self.txtRobot1_Z_WEIXUE,
                    self.txtRobot1_RX_WEIXUE,
                    self.txtRobot1_RY_WEIXUE,
                    self.txtRobot1_RZ_WEIXUE,
                ),
                tcp[:6],
            ):
                field.setText(str(round(value, 3)))

            state_text = {
                0: "伺服停止",
                1: "伺服就绪",
                2: "伺服报警",
                3: "伺服运行",
            }.get(servo_state, f"伺服未知({servo_state})")
            previous_servo_state = getattr(self, "_last_robot_servo_state", None)
            if servo_state == 2 and previous_servo_state != 2:
                self._teleop_event(
                    "error",
                    "CR5 进入伺服报警状态；请查看上方纳博特控制柜消息中的 code/message，"
                    "不要在故障原因未解除时反复清错上电",
                )
            self._last_robot_servo_state = servo_state
            self.robot1_message = (
                "Joint=" + ",".join(f"{value:.2f}" for value in joints) + "\n"
                "TCP=" + ",".join(f"{value:.3f}" for value in tcp) + "\n"
                + state_text
                + "\n"
            )
            self.txtRobotPackage.setPlainText(self.robot1_message)
            self._last_robot_poll_error = ""
        except Exception as exc:
            message = str(exc)
            if message != getattr(self, "_last_robot_poll_error", ""):
                self._last_robot_poll_error = message
                self._teleop_event("error", f"读取 CR5 状态失败: {message}")


    # GELLO
    def GELLO1_INIT(self):
        self.MasterConnect()

    def GELLO1_BIAS(self):
        self.txtGELLO1_BIAS_DATA.setText("GELLO 使用相对关节零点；开始跟随时自动对齐当前 CR3A")
        self._teleop_event("info", "GELLO 不强制回到旧初始姿态，启动跟随时自动建立当前主从相对零点")

    def GELLO1_FollowStart(self):
        self.TeleopFollowStart()

    def GELLO1_FollowStop(self):
        self.TeleopFollowStop()

    # 显示主手数据
    def on_timeout_render_gello(self):
        # GELLO 状态由 refresh_teleop_ui 统一显示。
        return

    # WEIXUE
    def on_timeout_render_weixue(self):
        return


    # 显示手数据
    def on_timeout_render_hand(self):
        global Hand_1_Pos
        global Hand_1_Error
        position, fault, _ = self.o6_controller.latest()
        if position is not None:
            Hand_1_Pos[:] = list(position)
        if fault is not None:
            Hand_1_Error[:] = list(fault)
        self.txtGripper1_Pos.setText(",".join(str(value) for value in Hand_1_Pos))
        self.txtGripper1_Vel.setText(",".join(str(value) for value in self.teleop_store.data["o6"]["speed"]))
        self.txtGripper1_Torque.setText(",".join(str(value) for value in self.teleop_store.data["o6"]["torque"]))
        self.txtGripper1_Error.setText(",".join(str(value) for value in Hand_1_Error))

    def closeEvent(self, event):
        """Never discard an episode or interrupt dataset finalization on exit."""
        dataset = self.lerobot_recorder.snapshot()
        if dataset["session_active"] and dataset["error"] and not self._episode_operation_lock.locked():
            event.ignore()
            if self.teleop_engine.state not in ("idle", "fault", "closed"):
                self._teleop_event("warning", "请先停止运动，再处理异常会话并退出；磁盘数据未清除")
                return
            answer = QMessageBox.warning(
                self, "保留异常数据并释放会话",
                f"采集会话出错：{dataset['error']}\n\n"
                f"是否保留 {dataset['root']} 中全部文件并释放采集进程？\n"
                "不会清除磁盘数据，但未完成数据可能不完整。完成后可再次关闭窗口。",
                QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel,
            )
            if answer == QMessageBox.Yes:
                self._run_episode_async("释放异常采集会话", self.lerobot_recorder.preserve_failed_session)
            return
        if (dataset["episode_active"] or dataset["buffered_frames"]
                or dataset["session_active"] or self._episode_operation_lock.locked()):
            event.ignore()
            self.tabWidget.setCurrentWidget(self.tab_4)
            self._teleop_event(
                "warning",
                "退出已取消：请先结束录制、保存或丢弃当前 Episode，再结束数据集。"
                "未清除任何数据；需要停止运动请按 F7 / 硬件急停。",
            )
            return
        for timer_name in ("timer_teleop_ui", "timer_render", "timer_D435_1", "timer_D435_2"):
            timer = getattr(self, timer_name, None)
            if timer is not None:
                timer.stop()
        try:
            self.teleop_engine.shutdown()
        except Exception as exc:
            self._teleop_event("error", f"退出时停止遥操失败：{exc}")
            event.ignore()
            self.timer_teleop_ui.start(100)
            return
        for name, thread in self._teleop_async_threads.items():
            if thread.is_alive():
                thread.join(timeout=5.0)
                if thread.is_alive():
                    event.ignore()
                    self._teleop_event("error", f"{name} 尚未退出，暂不关闭连接；请检查日志")
                    self.timer_teleop_ui.start(100)
                    return
        for label, device in (
            ("腕部", self.wrist_camera_device),
            ("基座", self.base_camera_device),
        ):
            try:
                device.close()
            except Exception as exc:
                self._teleop_event(
                    "error",
                    f"退出时{label}相机停止失败：{exc}",
                )
        self.D435_1_Started = False
        self.D435_2_Started = False
        robot_poll_thread = getattr(self, "_robot_poll_thread", None)
        if robot_poll_thread is not None and robot_poll_thread.is_alive():
            robot_poll_thread.join(timeout=2.0)
            if robot_poll_thread.is_alive():
                event.ignore()
                self._teleop_event("error", "NRC 状态线程未退出，暂不关闭 socket")
                self.timer_teleop_ui.start(100)
                return
        cr3a_device = getattr(self, "cr3a_device", None)
        if cr3a_device is not None:
            try:
                cr3a_device.close()
            except Exception as exc:
                self._teleop_event("error", f"退出时 CR3A 设备关闭失败：{exc}")
        self.cr3a_device = None
        self.nrc_adapter = None
        self.socketFd = -1
        self.socketFd_7000 = -1
        self.robot1_connected = False
        self.refresh_teleop_ui()
        for handler in self._file_logger.handlers:
            handler.close()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)

    myWin = MyMainForm()
    myWin.show()

    sys.exit(app.exec_())
