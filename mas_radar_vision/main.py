import threading
import time
from collections import deque
from information_ui import draw_information_ui
from fortify_logic import FortifyManager
import serial

import sys

import cv2
import numpy as np
from detect_function import YOLOv5Detector
from RM_serial_py.ser_api import  build_send_packet, receive_packet, \
    build_data_decision, build_data_radar_all
from rf_link import RfLinkClient
from rf_ui_panel import draw_rf_panel



'''赛场初始化设置'''
state       = 'B'                                               # R:红方/B:蓝方
camera_mode = 'video'                                           # 'test':测试模式(静态图片)，'hik':海康相机，'video':USB相机/videocapture（视频）
serial_mode = 'test'                                            # 'serial': 使用串口，'test': 禁用串口收发
com_set     = 'COM16'                                           # 串口，替换 'COM_' 为你的串口号

'''调试初始化设置'''
video_addr      = './_TestMaterial/Mas_Radar_Test_2605.avi'        #test视频地址
test_image_addr = 'images/test_image.jpg'



if state == 'R':
    loaded_arrays = np.load('arrays_test_red.npy')              # 加载标定好的仿射变换矩阵
    map_image = cv2.imread("mas_images/map_red.jpg")            # 加载红方视角地图
    mask_image = cv2.imread("mas_images/map_mask_red.jpg")      # 加载红方落点判断掩码
else:
    loaded_arrays = np.load('arrays_test_blue.npy')             # 加载标定好的仿射变换矩阵
    map_image = cv2.imread("mas_images/map_blue.jpg")           # 加载蓝方视角地图
    mask_image = cv2.imread("mas_images/map_mask_blue.jpg")     # 加载蓝方落点判断掩码

# 导入战场每个高度的不同仿射变化矩阵
M_ground        = loaded_arrays[0]                  # 地面层(黑)
M_height_center = loaded_arrays[1]                  # 中央高地(绿)
M_height_side   = loaded_arrays[2]                  # 侧面高地(蓝)
M_height_point  = loaded_arrays[3]                  # 制高点(红)


# 确定地图画面像素，保证不会溢出
height, width = mask_image.shape[:2]
height -= 1
width -= 1

# 初始化战场信息UI（标记进度、双倍易伤次数、双倍易伤触发状态）
information_ui = np.zeros((480, 1080, 3), dtype=np.uint8) * 255
information_ui_show = information_ui.copy()
Flag_require_vulnerability = False                              # 初始化双倍易伤标志位
double_vulnerability_chance = -1                                # 双倍易伤机会数
opponent_double_vulnerability = -1                              # 是否正在触发双倍易伤
remain_time = 420                                               # 七分钟
hp_tower_R = hp_tower_B = 1000                                  # 防御塔初始血量1000
target = -1                                                     # 飞镖当前瞄准目标（用于触发双倍易伤）
chances_flag = 1                                                # 双倍易伤触发标志位，需要从1递增，每小局比赛会重置，所以每局比赛要重启程序

vuln_request_active = False                                     # 请求-确认状态机（修 2026 版竞态）
fortify_cnt = 0                                                 # byte8 加固计数器，按小局清零
last_pw_cmd, last_pw_bytes = 0, b'000000'                       # 密钥字段暂存（SDR 侧破解成功时更新）
own_encryption_level = 1                                        # ←0x020E byte0 bit3-4
can_update_password = 0                                         # ←0x020E byte0 bit5
armor_fortify_chance = -1                                       # ←0x020E byte1 bit1-3
armor_fortify_active = -1                                       # ←0x020E byte1 bit0
fortify_circle = (0, 0, 0)                                      # ←0x020E byte2-7 生效加固圆 x,y,r(mm)
opp_jam_level = None                                            # 对方干扰难度：SDR 解调侧注入（用于压制门控）

# 加载战场地图
map_backup = cv2.imread("mas_images/map.jpg")
map = map_backup.copy()

# 初始化坐标猜测真值(即没识别到)
is_guest = {
    "B1": True,
    "B3": True,
    "B4": True,
    "B7": True,
    "R1": True,
    "R3": True,
    "R4": True,
    "R7": True
}

# 供发送的坐标列表(观测到的坐标, 无猜)
Tx_List_without_guess = {
    "R1": (0, 0),
    "R3": (0, 0),
    "R4": (0, 0),
    "R7": (0, 0),
    "B1": (0, 0),
    "B3": (0, 0),
    "B4": (0, 0),
    "B7": (0, 0)
}

# 当前标记状态
is_Locked = {
    "B1": 0,
    "B3": 0,
    "B4": 0,
    "B7": 0,
    "R1": 0,
    "R3": 0,
    "R4": 0,
    "R7": 0
}

# 机器人名字对应ID
id_dict = {
    "R1": 1,
    "R3": 3,
    "R4": 4,
    "R7": 7,
    "B1": 101,
    "B3": 103,
    "B4": 104,
    "B7": 107
}

# 盲区预测点位，如果没有定位模块，连接数服务器的非哨兵机器人坐标为（0,0）
# r侧为x轴0点
# guess_coord_list = {
#     "R1": [(1100, 1400), (900, 1400)],
#     "R2": [(870, 1100), (1340, 680)],
#     "R7": [(560, 630), (560, 870)],
#     "B1": [(1700, 100), (1900, 100)],
#     "B2": [(1930, 400), (1460, 820)],
#     "B7": [(2240, 870), (2240, 603)],
# }


# 机器人坐标滤波器（滑动窗口均值滤波）
class Filter:
    def __init__(self, window_size, max_inactive_time=2.0):
        self.window_size = window_size
        self.max_inactive_time = max_inactive_time
        self.data = {}  # 存储不同机器人的数据main.py
        self.window = {}  # 存储滑动窗口内的数据
        self.last_update_time = {}  # 存储每个机器人的最后更新时间

    # 添加机器人坐标数据
    def write_in_data(self, name, x, y, threshold=100000.0):  # 阈值单位为mm，实测没啥用，不如直接给大点
        global is_guest
        if name not in self.data:   # 如果机器人名称不在数据字典中，新建并初始化相应的deque滑动窗口。 
            self.data[name] = deque(maxlen=self.window_size)
            self.window[name] = deque(maxlen=self.window_size)
        '''
        if len(self.window[name]) >= 2:
            # 计算当前坐标与前一个坐标的均方
            msd = sum((a - b) ** 2 for a, b in zip((x, y), self.window[name][-1])) / 2.0
            # print(name, msd)

            if msd > threshold:
                # 如果均方差超过阈值，可能是异常值，不将其添加到数据中
                return
        '''
        # 将坐标数据添加到数据字典和滑动窗口中。
        self.data[name].append((x, y))
        is_guest[name] = False

        self.window[name].append((x, y))
        self.last_update_time[name] = time.time()  # 更新最后更新时间

    # 过滤计算滑动窗口平均值
    def filter_data(self, name):
        if name not in self.data:
            return None

        if len(self.window[name]) < self.window_size:
            return None  # 不足以进行滤波

        # 计算滑动窗口内的坐标平均值
        x_avg = sum(coord[0] for coord in self.window[name]) / self.window_size
        y_avg = sum(coord[1] for coord in self.window[name]) / self.window_size

        return x_avg, y_avg

    # 获取所有机器人坐标
    def get_all_detected_robot_data(self):
        filtered_d = {}
        for name in self.data:
            if time.time() - self.last_update_time[name] > self.max_inactive_time:  # 如果超时未检测到机器人
                self.data[name].clear()
                self.window[name].clear()
                is_guest[name] = True       # 清空相关数据并标注预测
            else:
                is_guest[name] = False
                filtered_d[name] = self.filter_data(name)
        # 返回所有当前识别到的机器人及其坐标的均值
        return filtered_d


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
    if state == 'R':
        set_Value(cam, param_type="float_value", node_name="ExposureTime", node_value=17000)  # 曝光时间 17000
    else:
        set_Value(cam, param_type="float_value", node_name="ExposureTime", node_value=17500)  # 曝光时间 17500
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
    cam = cv2.VideoCapture(video_addr)#位置
    while True:
        ret, img = cam.read()
        if ret:
            camera_image = img
            time.sleep(0.016)  # 60fps



# 串口发送线程
def ser_send():
    seq = 0
    global chances_flag, Flag_require_vulnerability, remain_time, hp_tower_R, hp_tower_B
    global isLocked_pre
    global ser1

    if ser1 is None:
        return 
    # 获取蓝方机器人坐标
    def get_coord_B(robot_name, robot_coord_pack):
        # front_time = time.time()
        # 转换为地图坐标系
        filtered_xyz = (2800 - robot_coord_pack[robot_name][1], robot_coord_pack[robot_name][0])
        # 转换为裁判系统单位M
        ser_x = int(filtered_xyz[0]) * 10 / 10
        ser_y = int(1500 - filtered_xyz[1]) * 10 / 10
        # 打包坐标数据包
        # data = build_data_radar(id_dict.get(robot_name), ser_x, ser_y)
        # packet, seq_s = build_send_packet(data, seq_s, [0x03, 0x05])
        # ser1.write(packet)
        # back_time = time.time()
        # # 计算发送延时，动态调整
        # waste_time = back_time - front_time
        # # print("发送：",robot_name, seq_s)
        # time.sleep(0.1 - waste_time)
        return ser_x,ser_y
    # 获取红方机器人坐标
    def get_coord_R(robot_name, robot_coord_pack):
        # front_time = time.time()
        # 转换为地图坐标系
        converted_coord = (robot_coord_pack[robot_name][1], 1500 - robot_coord_pack[robot_name][0])
        # 转换为裁判系统单位M
        ser_x = int(converted_coord[0]) * 10 / 10
        ser_y = int(1500 - converted_coord[1]) * 10 / 10
        # 打包坐标数据包
        # data = build_data_radar(id_dict.get(robot_name), ser_x, ser_y)
        # packet, seq_s = build_send_packet(data, seq_s, [0x03, 0x05])
        # ser1.write(packet)
        # back_time = time.time()
        # # 计算发送延时，动态调整
        # waste_time = back_time - front_time
        # # print('发送：',robot_name, seq_s)
        # time.sleep(0.1 - waste_time)
        return ser_x,ser_y

    # 获取盲区预测点坐标
    # 盲区预测已经彻底gg了
    ''' 
    time_s = time.time()
    target_last = 0  # 上一帧的飞镖目标
    update_time = 0  # 上次预测点更新时间
    '''
    send_count = 0  # 信道占用数，上限为4

    time_s = time.time()

    while True:
        guess_time_limit = 3 + 1.7  # 单位：秒，根据上一帧的信道占用数动态调整单点预测时间
        # print(guess_time_limit)
        send_count = 0  # 重置信道占用数
        try:
            # 1. 坐标获取部分
            all_filter_data = filter.get_all_detected_robot_data()
            if state == 'R':

                # 重装
                if is_guest.get('B1'):
                    Tx_List_without_guess['B1'] = (0, 0)
                else:
                    if all_filter_data.get('B1', False):
                        Tx_List_without_guess['B1'] = get_coord_B('B1', all_filter_data)

                # 步兵3号
                if not is_guest.get('B3'):
                    if all_filter_data.get('B3', False):
                        Tx_List_without_guess['B3'] = get_coord_B('B3',  all_filter_data)
                else:
                    Tx_List_without_guess['B3'] = (0, 0)

                # 步兵4号
                if not is_guest.get('B4'):
                    if all_filter_data.get('B4', False):
                        Tx_List_without_guess['B4'] = get_coord_B('B4', all_filter_data)
                else:
                    Tx_List_without_guess['B4'] = (0, 0)

                # 哨兵
                if is_guest.get('B7'):
                    Tx_List_without_guess['B7'] = (0, 0)
                else:
                    if all_filter_data.get('B7', False):
                        Tx_List_without_guess['B7'] = get_coord_B('B7', all_filter_data)

            if state == 'B':

                # 重装
                if is_guest.get('R1'):
                    Tx_List_without_guess['R1'] = (0, 0)
                else:
                    if all_filter_data.get('R1', False):
                        Tx_List_without_guess['R1'] = get_coord_R('R1', all_filter_data)

                # 步兵3号
                if not is_guest.get('R3'):
                    if all_filter_data.get('R3', False):
                        Tx_List_without_guess['R3'] = get_coord_R('R3', all_filter_data)
                else:
                    Tx_List_without_guess['R3'] = (0, 0)

                # 步兵4号
                if not is_guest.get('R4'):
                    if all_filter_data.get('R4', False):
                        Tx_List_without_guess['R4'] = get_coord_R('R4', all_filter_data)
                else:
                    Tx_List_without_guess['R4'] = (0, 0)

                # 哨兵
                if is_guest.get('R7'):
                    Tx_List_without_guess['R7'] = (0, 0)
                else:
                    if all_filter_data.get('R7', False):
                        Tx_List_without_guess['R7'] = get_coord_R('R7', all_filter_data)

            # 2. 坐标打包发送部分
            ser_data = build_data_radar_all(Tx_List_without_guess, state)       # 打包无猜坐标集
            Tx_packet, seq = build_send_packet(ser_data, seq, [0x03, 0x05])   # 0x0305 - 向裁判与所有己方队友广播机器人坐标
            print(Tx_packet)
            time.sleep(0.2)

            

            # 3. 双倍易伤释放部分 
            
            # 释放条件 (己方前哨站血量低于 500) || (敌方前哨站被击毁) || (已开局两分半钟(即剩余四分半钟))
            # 任意条件达到后，若有机会，将立即释放双倍易伤
            
            if Flag_require_vulnerability == False:
                if state == 'R':
                    if hp_tower_R < 500 or hp_tower_B <= 1 or remain_time <= 270:
                        Flag_require_vulnerability = True
                if state == 'B':
                    if hp_tower_B < 500 or hp_tower_R <= 1 or remain_time <= 270:
                        Flag_require_vulnerability = True

            if Flag_require_vulnerability == True:
                if double_vulnerability_chance > 0 and opponent_double_vulnerability == 0:
                    time_e = time.time()
                    # 发送时间间隔为10秒
                    if time_e - time_s > 10:
                        print("请求双倍易伤")
                        data = build_data_decision(chances_flag, state)
                        Tx_packet, seq = build_send_packet(data, seq, [0x03, 0x01])
                        #print(packet.hex(),chances_flag,state)
                        #safe_serial_write(Tx_packet)
                        print("请求成功", chances_flag)
                        # 更新标志位
                        chances_flag += 1
                        if chances_flag >= 3:
                            chances_flag = 1

                        time_s = time.time()

        except Exception as r:
            print('未知错误 %s' % (r))


# 裁判系统串口接收线程
def ser_receive():
    # 解析用数据
    global ser1                                 # 串口
    global remain_time                          # 剩余时间
    global hp_tower_R, hp_tower_B               # 前哨站血量
    global double_vulnerability_chance          # 拥有双倍易伤次数
    global opponent_double_vulnerability        # 双倍易伤触发状态
    progress_cmd_id         = [0x02, 0x0C]      # 机器人易伤标记的命令码0x020C
    vulnerability_cmd_id    = [0x02, 0x0E]      # 双倍易伤可用次数和触发状态0x020E
    global_status_cmd_id    = [0x00, 0x01]      # 全局状态列表，获取比赛阶段以及剩余时间
    hp_cmd_id               = [0x00, 0x03]      # 前哨站血量命令码
    buffer = b''                                # 初始化缓冲区

    while True:
        # 从串口读取数据
        if ser1 is not None and ser1.is_open:
            try:
                received_data = ser1.read_all()
            except serial.SerialException as e:
                print("[SERIAL WARN] E1004 read failed:", e)
                received_data = b''
        else:
            received_data = b''
        buffer += received_data

        # 查找帧头（SOF）的位置
        sof_index = buffer.find(b'\xA5')

        while sof_index != -1:
            # 如果找到帧头，尝试解析数据包
            if len(buffer) >= sof_index + 5:  # 至少需要5字节才能解析帧头
                # 从帧头开始解析数据包
                packet_data = buffer[sof_index:]

                # 查找下一个帧头的位置
                next_sof_index = packet_data.find(b'\xA5', 1)

                if next_sof_index != -1:
                    # 如果找到下一个帧头，说明当前帧头到下一个帧头之间是一个完整的数据包
                    packet_data = packet_data[:next_sof_index]
                    # print(packet_data)
                else:
                    # 如果没找到下一个帧头，说明当前帧头到末尾不是一个完整的数据包
                    break

                # 解析数据包
                mark_result = receive_packet(packet_data, progress_cmd_id, info=False)  # 解析单个数据包，cmd_id为0x020C,不输出日志
                vulnerability_result = receive_packet(packet_data, vulnerability_cmd_id, info=False)
                hp_result = receive_packet(packet_data, hp_cmd_id, info=False)
                global_status_result = receive_packet(packet_data, global_status_cmd_id, info=False)
                # 更新裁判系统数据，易伤标记、易伤可用次数、飞镖目标
                ''' 1. 标记状态获取 '''
                if mark_result is not None:
                    received_cmd_id1, received_data1, received_seq1 = mark_result
                    received_data1 = list(received_data1)[0]
                    if state == 'R':
                        is_Locked['B1'] = (received_data1 & 0b1) >> 0
                        is_Locked['B2'] = (received_data1 & 0b10) >> 1
                        is_Locked['B3'] = (received_data1 & 0b100) >> 2
                        is_Locked['B4'] = (received_data1 & 0b1000) >> 3
                        is_Locked['B7'] = (received_data1 & 0b10000) >> 4
                    else:
                        is_Locked['R1'] = (received_data1 & 0b1) >> 0
                        is_Locked['R2'] = (received_data1 & 0b10) >> 1
                        is_Locked['R3'] = (received_data1 & 0b100) >> 2
                        is_Locked['R4'] = (received_data1 & 0b1000) >> 3
                        is_Locked['R7'] = (received_data1 & 0b10000) >> 4

                ''' 2. 双倍易伤状态获取 '''
                if vulnerability_result is not None:
                    received_cmd_id2, received_data2, received_seq2 = vulnerability_result
                    received_data2 = list(received_data2)[0]

                    # 确保输入是一个字节
                    if not isinstance(received_data2, int) or received_data2 < 0 or received_data2 > 255:
                        raise ValueError("输入必须是一个字节（0-255）")

                    double_vulnerability_chance = received_data2 & 0b00000011
                    opponent_double_vulnerability = (received_data2 & 0b00000100) >> 2

                ''' 3. 前哨站血量获取 '''
                if hp_result is not None:
                    received_cmd_id3, received_data3, received_seq3 = hp_result
                    received_data3 = list(received_data3)
                    hp_tower_R = received_data3[12] + received_data3[13]*256
                    hp_tower_B = received_data3[28] + received_data3[29]*256

                ''' 4. 剩余时间获取 '''
                if global_status_result is not None:
                    received_cmd_id4, received_data4, received_seq4 = global_status_result
                    received_data4 = list(received_data4)
                    phase = (received_data4[0] & 0b11110000) >> 4
                    if phase == 4:
                        remain_time = received_data4[1] + received_data4[2]*256
                    else:
                        remain_time = 420

                # 从缓冲区中移除已解析的数据包
                buffer = buffer[sof_index + len(packet_data):]

                # 继续寻找下一个帧头的位置
                sof_index = buffer.find(b'\xA5')

            else:
                # 缓冲区中的数据不足以解析帧头，继续读取串口数据
                break
        time.sleep(0.5)


# 创建机器人坐标滤波器
filter = Filter(window_size=3, max_inactive_time=2)

# 加载模型，实例化机器人检测器和装甲板检测器
##weights_path = 'models/car.onnx'  # 建议把模型转换成TRT的engine模型
##weights_path_next = 'models/armor.onnx'
weights_path = 'models/car.engine'
weights_path_next = 'models/armor.engine'
detector_car = YOLOv5Detector(weights_path, data='yaml/car.yaml', conf_thres=0.1, iou_thres=0.5, max_det=14, ui=True)
detector_armor = YOLOv5Detector(weights_path_next, data='yaml/armor.yaml', conf_thres=0.50, iou_thres=0.2,
                                max_det=1,
                                ui=True)



# 串口接收线程
thread_receive = threading.Thread(target=ser_receive, daemon=True)
thread_receive.start()

# 串口发送线程
thread_list = threading.Thread(target=ser_send, daemon=True)
thread_list.start()

camera_image = None

# ===== A机 ↔ B机 有线UDP链路（2027 S99/S96；断链自动空操作，不影响雷达原功能）=====
def _set_opp_jam_level(level):
    global opp_jam_level
    opp_jam_level = level            # 供 ser_send 的 jam_ok 压制门控使用

def _set_opp_secret(key):
    global last_pw_cmd, last_pw_bytes
    last_pw_bytes = str(key).encode("ascii", "ignore")[:6]
    last_pw_cmd = 2                  # 协议表1-34：2=上报破解的对方密钥（1=改自己密钥）；上报后10s限速

fortify_mgr = FortifyManager()

rf_client = RfLinkClient(
    own_team=state,                  # 'R'/'B'，对方阵营自动取反
    fortify_mgr=fortify_mgr,         # hit 消息自动注入 feed_hit(x, y, zone)
    peer_host='192.168.3.20',        # 双机=场景4；单机同机联调(场景2)改 '127.0.0.1'
    peer_port=53201,
    listen_port=53200,
    on_jam_level=_set_opp_jam_level,
    on_secret_key=_set_opp_secret,
)
rf_client.start()

if serial_mode == 'serial':
    try:
        ser1 = serial.Serial(com_set, 115200, timeout=1)
        print("[SERIAL] COM opened")
    except serial.SerialException as e:
        print("[SERIAL FATAL] E1001 open failed:", e)
        sys.exit(1)
elif serial_mode == 'test':
    ser1 = None
    print("[SERIAL] test mode: serial disabled")


if camera_mode == 'test':
    camera_image = cv2.imread(test_image_addr)#位置
elif camera_mode == 'hik':
    # 海康相机图像获取线程
    from hik_camera import call_back_get_image, start_grab_and_get_data_size, close_and_destroy_device, set_Value, get_Value,image_control
    if sys.platform.startswith("win"):
        from MvImport.MvCameraControl_class import *
    else:
        from MvImport_Linux.MvCameraControl_class import *
    thread_camera = threading.Thread(target=hik_camera_get, daemon=True)
    thread_camera.start()
elif camera_mode == 'video':
    # USB相机图像获取线程
    thread_camera = threading.Thread(target=video_capture_get, daemon=True)
    thread_camera.start()

while camera_image is None:
    print("等待图像。。。")
    time.sleep(0.5)


# 获取相机图像的画幅，限制点不超限
img0 = camera_image.copy()
img_y = img0.shape[0]
img_x = img0.shape[1]
print(img0.shape)

while True:
    # 刷新裁判系统信息UI图像
    information_ui_show = information_ui.copy()
    map = map_backup.copy()
    det_time = 0
    img0 = camera_image.copy()

    #img0_show=cv2.resize(img0, (900, 480))

    #cv2.imshow("img_show",img0_show)

    ts = time.time()
    # 第一层神经网络识别
    result0 = detector_car.predict(img0)
    det_time += 1
    for detection in result0:
        cls, xywh, conf = detection
        if cls == 'car':
            left, top, w, h = xywh
            left, top, w, h = int(left), int(top), int(w), int(h)  
            # 存储第一次检测结果和区域
            # ROI出机器人区域
            cropped = camera_image[top:top + h, left:left + w]
            cropped_img = np.ascontiguousarray(cropped)
            # 第二层神经网络识别
            result_n = detector_armor.predict(cropped_img)
            det_time += 1
            if result_n:
                # 叠加第二次检测结果到原图的对应位置
                img0[top:top + h, left:left + w] = cropped_img

                for detection1 in result_n:
                    cls, xywh, conf = detection1
                    if cls:  # 所有装甲板都处理，可选择屏蔽一些:
                        x, y, w, h = xywh
                        x = x + left
                        y = y + top

                        t1 = time.time() 
                        # 原图中装甲板的中心下沿作为待仿射变化的点
                        # 以实际位置对比，发现-1.0*w与1.0*h误差比较小。实际上，该参数与摄像头摆放位姿强相关。
                        camera_point = np.array([[[min(x - 1.0*w, img_x), min(y + 1.0*h, img_y)]]],
                                                dtype=np.float32)
                        # 低到高依次仿射变化
                        # 先套用地面层仿射变化矩阵
                        mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_ground)
                        # 限制转换后的点在地图范围内
                        x_c = max(int(mapped_point[0][0][0]), 0)
                        y_c = max(int(mapped_point[0][0][1]), 0)
                        x_c = min(x_c, width)
                        y_c = min(y_c, height)
                        color = mask_image[y_c, x_c]  # 通过掩码图像，获取地面层的颜色：黑（0，0，0）
                        X_M_1 = x_c
                        Y_M_1 = y_c
                        if color[0] < 5 and color[1] < 5 and color[2] < 5:
                            X_M = x_c
                            Y_M = y_c
                            # Z_M = 0
                            filter.write_in_data(cls, X_M, Y_M)
                        else:
                            # 不满足则继续套用侧面高地层仿射变换矩阵
                            mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_height_side)
                            # 限制转换后的点在地图范围内
                            x_c = max(int(mapped_point[0][0][0]), 0)
                            y_c = max(int(mapped_point[0][0][1]), 0)
                            x_c = min(x_c, width)
                            y_c = min(y_c, height)
                            color = mask_image[y_c, x_c]  # 通过掩码图像，获取侧面高地层的颜色：蓝（255，0，0）
                            if color[0] > color[2] and color[0] > color[1]:
                                X_M = x_c
                                Y_M = y_c
                                # Z_M = 200
                                filter.write_in_data(cls, X_M, Y_M)
                            else:
                                # 不满足则继续套用中央高地层仿射变换矩阵
                                mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_height_center)
                                # 限制转换后的点在地图范围内
                                x_c = max(int(mapped_point[0][0][0]), 0)
                                y_c = max(int(mapped_point[0][0][1]), 0)
                                x_c = min(x_c, width)
                                y_c = min(y_c, height)
                                color = mask_image[y_c, x_c]  # 通过掩码图像，获取中央高地层的颜色：绿（0，255，0）
                                if color[1] > color[2] and color[1] > color[0]:
                                    X_M = x_c
                                    Y_M = y_c
                                    # Z_M = 300
                                    filter.write_in_data(cls, X_M, Y_M)
                                else:
                                    # 不满足则继续套用制高点层仿射变换矩阵
                                    mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_height_point)
                                    # 限制转换后的点在地图范围内
                                    x_c = max(int(mapped_point[0][0][0]), 0)
                                    y_c = max(int(mapped_point[0][0][1]), 0)
                                    x_c = min(x_c, width)
                                    y_c = min(y_c, height)
                                    color = mask_image[y_c, x_c]  # 通过掩码图像，获取制高点层的颜色：红（0，0，255）
                                    if color[2] > color[0] and color[2] > color[1]:
                                        X_M = x_c
                                        Y_M = y_c
                                        # Z_M = 650
                                        filter.write_in_data(cls, X_M, Y_M)
                                    else:
                                        filter.write_in_data(cls, X_M_1, Y_M_1)
                                    
    # 获取所有识别到的机器人坐标  
    all_filter_data = filter.get_all_detected_robot_data()
    # print(all_filter_data_name)
    if all_filter_data != {}:
        for name, xyxy in all_filter_data.items():
            if xyxy is not None:
                if name[0] == "R":
                    color_m = (0, 0, 255)
                else:
                    color_m = (255, 0, 0)
                if state == 'R':
                    filtered_xyz = (2800 - xyxy[1], xyxy[0])  # 缩放坐标到地图图像
                    ##print(name,filtered_xyz)
                else:
                    filtered_xyz = (xyxy[1], 1500 - xyxy[0])  # 缩放坐标到地图图像
                    ##print(name,filtered_xyz)
                # 只绘制敌方阵营的机器人（这里不会绘制盲区预测的机器人）
                #if name[0] != state:
                cv2.circle(map, (int(filtered_xyz[0]), int(filtered_xyz[1])), 15, color_m, -1)  # 绘制圆
                cv2.putText(map, str(name),
                                (int(filtered_xyz[0]) - 5, int(filtered_xyz[1]) + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 5)
                ser_x = int(filtered_xyz[0]) * 10 / 10
                ser_y = int(1500 - filtered_xyz[1]) * 10 / 10
                cv2.putText(map, "(" + str(ser_x) + "," + str(ser_y) + ")",
                                (int(filtered_xyz[0]) - 100, int(filtered_xyz[1]) + 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)

    te = time.time()
    fps = round(1/(te - ts), 2)

    if state == 'B':
        cv2.putText(information_ui_show, "TEAM [" + str(state) + "]",
                    (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0xFF, 0xDF, 0x00), 3)
        cv2.putText(information_ui_show, "Coord Information:",
                    (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0xFF, 0xDF, 0x00), 2)
        cv2.putText(information_ui_show, "R1: " + str(Tx_List_without_guess['R1']),
                    (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0x00, 0x00, 0xFF), 2)
        cv2.putText(information_ui_show, "R2: " + str(Tx_List_without_guess['R3']),
                    (220, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0x00, 0x00, 0xFF), 2)
        cv2.putText(information_ui_show, "R3: " + str(Tx_List_without_guess['R4']),
                    (430, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0x00, 0x00, 0xFF), 2)
        cv2.putText(information_ui_show, "R4: " + str(Tx_List_without_guess['R7']),
                    (640, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0x00, 0x00, 0xFF), 2)
        cv2.putText(information_ui_show, "Mark Status:",
                    (10, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0xFF, 0xDF, 0x00), 2)
        cv2.putText(information_ui_show, "R1: " + str(is_Locked['R1']),
                    (10, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0x00, 0x00, 0xFF), 2)
        cv2.putText(information_ui_show, "R2: " + str(is_Locked['R3']),
                    (115, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0x00, 0x00, 0xFF), 2)
        cv2.putText(information_ui_show, "R3: " + str(is_Locked['R4']),
                    (220, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0x00, 0x00, 0xFF), 2)
        cv2.putText(information_ui_show, "R4: " + str(is_Locked['R7']),
                    (325, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0x00, 0x00, 0xFF), 2)
        cv2.putText(information_ui_show, "Our Tower HP: " + str(hp_tower_B),
                    (10, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0xFF, 0xDF, 0x00), 2)
        cv2.putText(information_ui_show, "Enemy Tower HP: " + str(hp_tower_R),
                    (310, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0x00, 0x00, 0xFF), 2)
    else:
        cv2.putText(information_ui_show, "TEAM [" + str(state) + "]",
                    (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0x00, 0x00, 0xFF), 3)
        cv2.putText(information_ui_show, "Coord Information:",
                    (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0x00, 0x00, 0xFF), 2)
        cv2.putText(information_ui_show, "B1: " + str(Tx_List_without_guess['B1']),
                    (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0xFF, 0xDF, 0x00), 2)
        cv2.putText(information_ui_show, "B2: " + str(Tx_List_without_guess['B3']),
                    (220, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0xFF, 0xDF, 0x00), 2)
        cv2.putText(information_ui_show, "B3: " + str(Tx_List_without_guess['B4']),
                    (430, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0xFF, 0xDF, 0x00), 2)
        cv2.putText(information_ui_show, "B4: " + str(Tx_List_without_guess['B7']),
                    (640, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0xFF, 0xDF, 0x00), 2)
        cv2.putText(information_ui_show, "Mark Status:",
                    (10, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0x00, 0x00, 0xFF), 2)
        cv2.putText(information_ui_show, "B1: " + str(is_Locked['B1']),
                    (10, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0xFF, 0xDF, 0x00), 2)
        cv2.putText(information_ui_show, "B2: " + str(is_Locked['B3']),
                    (115, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0xFF, 0xDF, 0x00), 2)
        cv2.putText(information_ui_show, "B3: " + str(is_Locked['B4']),
                    (220, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0xFF, 0xDF, 0x00), 2)
        cv2.putText(information_ui_show, "B4: " + str(is_Locked['B7']),
                    (325, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0xFF, 0xDF, 0x00), 2)
        cv2.putText(information_ui_show, "Our Tower HP: " + str(hp_tower_R),
                    (10, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0x00, 0x00, 0xFF), 2)
        cv2.putText(information_ui_show, "Enemy Tower HP: " + str(hp_tower_B),
                    (310, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0xFF, 0xDF, 0x00), 2)

    cv2.putText(information_ui_show, "remain time: " + str(remain_time),
                (10, 320),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0xFF, 0xFF, 0xFF), 2)
    cv2.putText(information_ui_show, "current vulnerability chances: " + str(double_vulnerability_chance),
                (10, 360),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0xFF, 0xFF, 0xFF), 2)
    cv2.putText(information_ui_show, "vulnerability using status: " + str(opponent_double_vulnerability),
                (10, 400),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0xFF, 0xFF, 0xFF), 2)
    cv2.putText(information_ui_show, "fps: " + str(fps),
                (10, 440),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0xFF, 0xFF, 0xFF), 2)

    img0 = cv2.resize(img0, (1300, 900))
    cv2.imshow('img', img0)
    information_ui_show = np.vstack((information_ui_show, draw_rf_panel(rf_client)))
    cv2.imshow('information_ui', information_ui_show)
    map_show = cv2.resize(map, (600, 320))
    #mas_show= cv2.resize(mas, (600, 320))
    cv2.imshow('map', map_show)
    #cv2.imshow('mas', mas_show)
    #img0 = cv2.resize(img0, (1300, 900))
                      
    #cv2.namedWindow("img",cv2.WINDOW_AUTOSIZE)


    key = cv2.waitKey(1)
