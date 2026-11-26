import threading
import time

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage, QTextCursor
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QTextEdit, QGridLayout

import sys


# 海康相机图像获取线程
def hik_camera_get():
    # 获得设备信息
    global camera_image
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # ch:枚举设备 | en:Enum device
    # nTLayerType [IN] 枚举传输层 ，pstDevList [OUT] 设备列表
    while 1:
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            print("enum devices fail! ret[0x%x]" % ret)
            # sys.exit()

        if deviceList.nDeviceNum == 0:
            print("find no device!")
            # sys.exit()
        else:
            print("Find %d devices!" % deviceList.nDeviceNum)
            break

    for i in range(0, deviceList.nDeviceNum):
        mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
            print("\ngige device: [%d]" % i)
            # 输出设备名字
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)
            # 输出设备ID
            nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
            nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
            nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
            nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
            print("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
        # 输出USB接口的信息
        elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
            print("\nu3v device: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)

            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            print("user serial number: %s" % strSerialNumber)
    # 手动选择设备
    # nConnectionNum = input("please input the number of the device to connect:")
    # 自动选择设备
    nConnectionNum = '0'
    if int(nConnectionNum) >= deviceList.nDeviceNum:
        print("intput error!")
        sys.exit()

    # ch:创建相机实例 | en:Creat Camera Object
    cam = MvCamera()

    # ch:选择设备并创建句柄 | en:Select device and create handle
    # cast(typ, val)，这个函数是为了检查val变量是typ类型的，但是这个cast函数不做检查，直接返回val
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

    ret = cam.MV_CC_CreateHandle(stDeviceList)
    if ret != 0:
        print("create handle fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:打开设备 | en:Open device
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("open device fail! ret[0x%x]" % ret)
        sys.exit()

    print(get_Value(cam, param_type="float_value", node_name="ExposureTime"),
          get_Value(cam, param_type="float_value", node_name="Gain"),
          get_Value(cam, param_type="enum_value", node_name="TriggerMode"),
          get_Value(cam, param_type="float_value", node_name="AcquisitionFrameRate"))

    # 设置设备的一些参数
    set_Value(cam, param_type="float_value", node_name="ExposureTime", node_value=16000)  # 曝光时间
    set_Value(cam, param_type="float_value", node_name="Gain", node_value=17.9)  # 增益值
    # 开启设备取流
    start_grab_and_get_data_size(cam)
    # 主动取流方式抓取图像
    stParam = MVCC_INTVALUE_EX()

    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
    ret = cam.MV_CC_GetIntValueEx("PayloadSize", stParam)
    if ret != 0:
        print("get payload size fail! ret[0x%x]" % ret)
        sys.exit()
    nDataSize = stParam.nCurValue
    pData = (c_ubyte * nDataSize)()
    stFrameInfo = MV_FRAME_OUT_INFO_EX()

    memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
    while True:
        ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
        if ret == 0:
            image = np.asarray(pData)
            # 处理海康相机的图像格式为OPENCV处理的格式
            camera_image = image_control(data=image, stFrameInfo=stFrameInfo)
        else:
            print("no data[0x%x]" % ret)


def video_capture_get():
    global camera_image
    # cam = cv2.VideoCapture('.\_TestMaterial\__Radar_Test_Video.avi')
    cam = cv2.VideoCapture('./_TestMaterial/Mas_Radar_Test_2602.avi')
    while True:
        ret, img = cam.read()
        if ret:
            camera_image = img
            time.sleep(0.016)  # 60fps


color = [(255, 255, 255), (0, 255, 0), (0, 0, 255),(255, 0, 0)]                 #平面图标定颜色
mask_color = [(255,255,255), (255, 0, 255), (255, 255, 0),(0, 255, 255)]        # mask 图显示颜色（相反色）
ROBO_HEIGHT = 4                                                                 #标定层数
cal_state = 'B'                                                                 # R:红方/B:蓝方
cal_camera_mode = 'video'                                                       # 'test':测试模式,'hik':海康相机,'video':USB相机（videocapture）

class MyUI(QWidget):
    def __init__(self):
        super().__init__()
        self.capturing = True
        self.initUI()

    def initUI(self):
        from PyQt5.QtWidgets import QSizePolicy  # 若文件顶部已导入可删掉这行

        # 左上角部分（视频，位置和大小不变）
        self.state = state
        self.left_top_label = QLabel(self)
        self.left_top_label.setFixedSize(1350, 1000)
        self.left_top_label.setStyleSheet("border: 2px solid black;")
        self.left_top_label.mousePressEvent = self.left_top_clicked
        self.image_points = [[(0, 0)] * 4 for _ in range(ROBO_HEIGHT)]
        self.map_points = [[(0, 0)] * 4 for _ in range(ROBO_HEIGHT)]
        self.image_count = 0
        self.map_count = 0

        # 右上角第一张图片（地图）
        self.right_top_label = QLabel(self)
        self.right_top_label.setFixedSize(550, 900)
        self.right_top_label.setStyleSheet("border: 2px solid black;")
        self.right_top_label.mousePressEvent = self.right_top_clicked

        # 右上角第二张图片（mask）
        self.mask_label = QLabel(self)
        self.mask_label.setFixedSize(550, 900)
        self.mask_label.setStyleSheet("border: 2px solid black;")

        # 第二张图片下方的文本框（日志）
        self.left_bottom_text = QTextEdit(self)
        self.left_bottom_text.setStyleSheet("font-size: 18px; border: 2px solid black;")

        # 按钮
        self.button1 = QPushButton('开始标定', self)
        self.button1.clicked.connect(self.button1_clicked)
        self.button2 = QPushButton('切换高度', self)
        self.button2.clicked.connect(self.button2_clicked)
        self.button3 = QPushButton('加载坐标', self)
        self.button3.clicked.connect(self.button3_clicked)
        self.button4 = QPushButton('保存计算', self)
        self.button4.clicked.connect(self.button4_clicked)

        self.height = 0
        self.T = []
        if self.state == 'R':
            self.save_path = 'arrays_test_red.npy'
            right_image_path = "mas_images/map_red.jpg"
            mask_image_path = "mas_images/map_mask_adv_red.jpg"
        else:
            self.save_path = 'arrays_test_blue.npy'
            right_image_path = "mas_images/map_blue.jpg"
            mask_image_path = "mas_images/map_mask_adv_blue.jpg"

        left_image = camera_image
        right_image = cv2.imread(right_image_path)
        mask_image = cv2.imread(mask_image_path)

        self.left_scale_x = left_image.shape[1] / 1350.0
        self.left_scale_y = left_image.shape[0] / 1000.0
        self.right_scale_x = right_image.shape[1] / 550.0
        self.right_scale_y = right_image.shape[0] / 900.0

        left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)
        self.left_image = cv2.resize(left_image, (1350, 1000))
        right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB)
        self.right_image = cv2.resize(right_image, (550, 900))
        self.mask_image = cv2.cvtColor(mask_image, cv2.COLOR_BGR2RGB)
        self.mask_image = cv2.resize(self.mask_image, (550, 900))
        self.mask_color = mask_color
        self.update_images()

        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera)
        self.camera_timer.start(50)

        # 按钮样式 + 可拉伸
        for b in (self.button1, self.button2, self.button3, self.button4):
            self.set_button_style(b)

        # 2×2 按钮，占满第一张图下方
        grid_layout = QGridLayout()
        grid_layout.addWidget(self.button1, 0, 0)
        grid_layout.addWidget(self.button2, 0, 1)
        grid_layout.addWidget(self.button3, 1, 0)
        grid_layout.addWidget(self.button4, 1, 1)
        grid_layout.setSpacing(10)
        grid_layout.setContentsMargins(0, 0, 0, 0)
        grid_layout.setRowStretch(0, 1)
        grid_layout.setRowStretch(1, 1)
        grid_layout.setColumnStretch(0, 1)
        grid_layout.setColumnStretch(1, 1)

        buttons_widget = QWidget()
        buttons_widget.setLayout(grid_layout)
        buttons_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # 右列1：第一张图 + 按钮区
        vbox_right_first = QVBoxLayout()
        vbox_right_first.setSpacing(10)
        vbox_right_first.setContentsMargins(0, 0, 0, 0)
        vbox_right_first.addWidget(self.right_top_label)
        vbox_right_first.addWidget(buttons_widget, 1)

        # 右列2：第二张图 + 文本框
        vbox_right_second = QVBoxLayout()
        vbox_right_second.setSpacing(10)
        vbox_right_second.setContentsMargins(0, 0, 0, 0)
        vbox_right_second.addWidget(self.mask_label)
        vbox_right_second.addWidget(self.left_bottom_text, 1)

        right_images_widget = QWidget()
        hbox_right_images = QHBoxLayout(right_images_widget)
        hbox_right_images.setSpacing(10)
        hbox_right_images.setContentsMargins(0, 0, 0, 0)
        hbox_right_images.addLayout(vbox_right_first, 1)
        hbox_right_images.addLayout(vbox_right_second, 1)

        # 左侧：视频 + "标定区域"大字
        self.left_bottom_text_big = QLabel('标定时应按照mask颜色标定对应区域', self)
        self.left_bottom_text_big.setAlignment(Qt.AlignCenter)
        self.left_bottom_text_big.setStyleSheet(
            "font-size: 50px; font-weight: bold; border: 2px solid black;")

        vbox_left = QVBoxLayout()
        vbox_left.setSpacing(10)
        vbox_left.setContentsMargins(0, 0, 0, 0)
        vbox_left.addWidget(self.left_top_label)
        vbox_left.addWidget(self.left_bottom_text_big, 1)

        hbox = QHBoxLayout()
        hbox.setSpacing(10)
        hbox.addLayout(vbox_left, 0)
        hbox.addWidget(right_images_widget, 0)

        self.setLayout(hbox)
        self.setGeometry(0, 0, 2500, 1000)   # 高度只延伸到 1250
        self.setWindowTitle('Calibration UI')
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.show()

    def keyPressEvent(self, event):
        # 按下键盘事件
        if event.key() == Qt.Key_Escape:
            self.close()

    def update_images(self):
        left_pixmap = self.convert_cvimage_to_pixmap(self.left_image)
        self.left_top_label.setPixmap(left_pixmap)

        right_pixmap = self.convert_cvimage_to_pixmap(self.right_image)
        self.right_top_label.setPixmap(right_pixmap)

        mask_pixmap = self.convert_cvimage_to_pixmap(self.mask_image)
        self.mask_label.setPixmap(mask_pixmap)

    def update_camera(self):
        if self.capturing:
            img0 = camera_image
            left_image = cv2.cvtColor(img0, cv2.COLOR_BGR2RGB)
            self.left_image = cv2.resize(left_image, (1350, 1000))
            self.update_images()

    def left_top_clicked(self, event):
        # 图像点击事件
        if not self.capturing:
            x = int(event.pos().x() * self.left_scale_x)
            y = int(event.pos().y() * self.left_scale_y)

            self.image_points[self.height][self.image_count % 4] = (x, y)

            cv2.circle(self.left_image, (int(x / self.left_scale_x), int(y / self.left_scale_y)), 4, color[self.height],
                       -1)
            cv2.putText(self.left_image, str(self.image_count % 4),
                        (int(x / self.left_scale_x), int(y / self.left_scale_y)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color[self.height], 3)
            
            self.image_count += 1
            self.update_images()
            self.append_text(f'图像真实点击坐标：({x}, {y})')

    def right_top_clicked(self, event):
        # 地图点击事件
        if not self.capturing:
            x = int(event.pos().x() * self.right_scale_x)
            y = int(event.pos().y() * self.right_scale_y)
            self.map_points[self.height][self.map_count % 4] = (x, y)

            cv2.circle(self.right_image, (int(x / self.right_scale_x), int(y / self.right_scale_y)), 4,
                       color[self.height],
                       -1)
            cv2.putText(self.right_image, str(self.map_count % 4),
                        (int(x / self.right_scale_x), int(y / self.right_scale_y)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color[self.height], 2)
            # mask 图同步画点，用当前高度的相反色
            mc = self.mask_color[self.height]
            cv2.circle(self.mask_image, (int(x / self.right_scale_x), int(y / self.right_scale_y)), 4, mc, -1)
            cv2.putText(self.mask_image, str(self.map_count % 4),
                        (int(x / self.right_scale_x), int(y / self.right_scale_y)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, mc, 2)

            self.map_count += 1
            self.update_images()
            self.append_text(f'地图真实点击坐标：({x}, {y})')

    def button1_clicked(self):
        # 按钮1点击事件
        self.append_text('开始标定')
        self.capturing = False

        print('开始标定')

    def button2_clicked(self):
        # 按钮2点击事件
        self.append_text('切换高度')
        self.image_count = 0
        self.map_count = 0
        self.height = (self.height + 1) % ROBO_HEIGHT
        print('切换高度')

    def button3_clicked(self):                      ##开发撤销按键
        # 按钮3点击事件
        self.append_text('N/A')  # 该功能还未制作

        print('N/A')

    def button4_clicked(self):
        # 按钮4点击事件
        print(self.image_points)
        print(self.map_points)
        for i in range(0, ROBO_HEIGHT):
            image_point = np.array(self.image_points[i], dtype=np.float32)
            map_point = np.array(self.map_points[i], dtype=np.float32)
            self.T.append(cv2.getPerspectiveTransform(image_point, map_point))

        np.save(self.save_path, self.T)

        self.append_text('保存计算')
        print('保存计算', self.save_path)
        time.sleep(1)
        sys.exit()

    def convert_cvimage_to_pixmap(self, cvimage):
        height, width, channel = cvimage.shape
        bytes_per_line = 3 * width
        qimage = QImage(cvimage.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        return pixmap

    def set_button_style(self, button):
        button.setStyleSheet("QPushButton { font-size: 20px; }")

    def append_text(self, text):
        # 在文本组件中追加文本
        current_text = self.left_bottom_text.toPlainText()
        self.left_bottom_text.setPlainText(current_text + '\n' + text)
        # 自动向下滚动文本组件
        cursor = self.left_bottom_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.left_bottom_text.setTextCursor(cursor)


if __name__ == '__main__':
    camera_mode = cal_camera_mode  
    camera_image = None
    state = cal_state  

    if camera_mode == 'test':
        camera_image = cv2.imread('images/test_image.jpg')
    elif camera_mode == 'hik':
        # 海康相机图像获取线程
        from hik_camera import call_back_get_image, start_grab_and_get_data_size, close_and_destroy_device, set_Value, \
            get_Value, image_control

        if sys.platform.startswith("win"):
            from MvImport.MvCameraControl_class import *

            print("win")
        else:
            from MvImport_Linux.MvCameraControl_class import *

            print('linux')
        thread_camera = threading.Thread(target=hik_camera_get, daemon=True)
        thread_camera.start()
    elif camera_mode == 'video':
        # USB相机图像获取线程
        thread_camera = threading.Thread(target=video_capture_get, daemon=True)
        thread_camera.start()

    while camera_image is None:
        print("等待图像。。。")
        time.sleep(0.5)
    app = QApplication(sys.argv)
    myui = MyUI()
    sys.exit(app.exec_())
