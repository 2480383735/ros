#!/usr/bin/env python
#coding=utf-8
import rospy
import sys
import glob
import threading
import binascii
import serial
import geometry_msgs.msg
import amc_base_msg.msg
import math
import time

# LZ-600
# length      = 270
# width       = 194
# wheeltrack  = 74

# LZ-PRO
# length      = 300
# width       = 236.5
# wheeltrack  = 64

#LZ-STD
length      = 210
width       = 151
wheeltrack  = 63

# 速度最大值 mm/s
vel_max =   1000
# 角度最大值 °
ang_max =   20

# 类型转换无符号整形转化成有符号整形
def type_conversion(data, len):
    if data & (1 << (len * 8 - 1)):
        data = data - (1 << (len * 8))
    return data

# 计算校验和,返回校验结果
def check_sum(data):
    data_check = 0
    for i in range(2, len(data)-1):
        data_check ^= data[i]
        data_check = data_check & 0xFF
    return data_check

# 串口连接 默认连接第一个串口
def serial_connect(serialName):
    command = "/dev/" + serialName
    # 搜索串口
    serial_name  = glob.glob(command)
    # 无可用串口时退出程序
    if len(serial_name) == 0:
        rospy.logerr("无可用串口")
        sys.exit()
    # 使用第一个USB口作为串口地址
    rospy.loginfo(serial_name[0])
    SerialPort = serial.Serial(serial_name[0], 115200, timeout=0.5)
    rospy.loginfo("连接成功")
    return SerialPort

data = ""
# 获取串口数据校验并返回
def get_serial_data():
    # 接收python3数据
    data_hex = []
    data_return = []
    global data
    if sys.version[0] == '3':
        
        if SerialPort.in_waiting > 0:
            # 读取数据bytes
            data = SerialPort.read_all().hex()
           
    else:
        if SerialPort.in_waiting > 0:
            data = SerialPort.read_all().encode('hex')
    b = []
    c = []
    d = data
    for i in range(0, len(d)):
        if d[i:i + 4] == '0a0c':
            b.append(i)
    # noinspection PyBroadException
    try:
        for i in range(0, len(b)):
            if check_sum_hex(d[b[i]: b[i] + int(d[b[i] + 6:b[i] + 8] + d[b[i] + 4:b[i] + 6], 16) * 2 + 10]) == int(d[b[i] + int(d[b[i] + 6:b[i] + 8] + d[b[i] + 4:b[i] + 6], 16) * 2 + 10:b[i] + int(d[b[i] + 6:b[i] + 8] + d[b[i] + 4:b[i] + 6], 16) * 2 + 12], 16):
                c.append(d[b[i]:b[i] + int(d[b[i] + 6:b[i] + 8] + d[b[i] + 4:b[i] + 6], 16) * 2 + 12])
    except Exception as e:
        pass
    for e in c:
      for i in range(0, len(e),2):
        data_return.append(int(e[i:i+2], 16))
      data_hex.append(data_return)
      data_return = []
    return data_hex
    
def check_sum_hex(command):
    if command[4:]:
        a = int(command[4:6], 16)
        for i in range(6, len(command), 2):
            a ^= int(command[i:i+2], 16)
        return a
            
# 发布bms信息
def publish_bms_info(pub, data):
    bms_data = amc_base_msg.msg.bms()
    # 时间戳
    bms_data.stamp=rospy.Time.now()
    # 电池容量%
    bms_data.volume_Percentage = type_conversion(data[5] | (data[6] << 8), 2)
    # 电池电流A
    bms_data.current_A = type_conversion(data[7] | (data[8] << 8), 2) / 100.0
    # 电池电压V
    bms_data.voltage_V = type_conversion(data[9] | (data[10] << 8), 2) / 100.0
    pub.publish(bms_data)

# 发布速度信息
def publish_speed_info(pub, data, cmd):
    speed_data = amc_base_msg.msg.car_speed()
    if cmd == 0x6A:
        # 时间戳
        speed_data.stamp=rospy.Time.now()
        # 线速度
        speed_data.linear = type_conversion(data[5] | (data[6] << 8), 2) / 1000.0
        # # 角速度
        speed_data.angular = type_conversion(data[9] | (data[10] << 8), 2) / 1000.0
    pub.publish(speed_data)
        

# 数据处理
def data_processing(data, speed_pub, bms_pub):
    try:
       
        if len(data) > 6 and data[0] == 0x0A and data[1] == 0x0C and data[-1] == check_sum(data):
            cmd = data[4]
            if (cmd == 0x6A) and len(data) == 16:
                    publish_speed_info(speed_pub, data, cmd)
                    
            elif cmd == 0x69 and len(data) == 14:
                publish_bms_info(bms_pub, data)
        else:
            for i in range(len(data)):
                data[i] = format(data[i], 'X').zfill(2)
            rospy.logwarn(data)
    except Exception as e:
        
        for i in range(len(data)):
                data[i] = format(data[i], 'X').zfill(2)
        rospy.logerr(data)

# 使能
def power_on_machine():
    SerialPort.write(binascii.a2b_hex('0A'+'0C'+'01'+'00'+'01'+'01'+'01'))
    
# 失能
def power_off_machine():
    SerialPort.write(binascii.a2b_hex('0A'+'0C'+'01'+'00'+'01'+'00'+'00'))

# 接收数据线程
def data_receive_thread():
    # 创建BMS话题
    bms_pub = rospy.Publisher("/bms", amc_base_msg.msg.bms, queue_size=1)
    # 创建速度话题
    speed_pub = rospy.Publisher("/car_speed", amc_base_msg.msg.car_speed, queue_size=1)
    while not rospy.is_shutdown():
        try:
            # 获取数据
            data = get_serial_data()
        except Exception as e:
            rospy.logerr(e)
            rospy.logwarn("串口连接断开！")
            rospy.signal_shutdown("串口连接断开！")
        
        if data != None:
            for data_hex in data:
            # 数据处理
                data_processing(data_hex, speed_pub, bms_pub)
    
# 处理用户下发数据
def handle_data(data):
    # 获取线速度，转换成mm/s
    linear = round(data.linear.x * 1000)
    # 获取角速度
    angular = data.angular.z

    # 线速度角速度指令
    # angular = round(angular * 1000)
    # data = [0x0A, 0x0C, 0x04, 0x00, 0x06, linear & 0xFF, (linear >> 8) & 0xFF, angular & 0xFF, (angular >> 8) & 0xFF, 0x00]

###############################################################################################################################
    # 转换成偏航角速度模式
    # 原地转
    if linear == 0 and angular != 0:
        round_vel = (math.sqrt(length*length + width*width)+wheeltrack) * angular
        if round_vel > vel_max:
            round_vel = vel_max
        elif round_vel < -vel_max:
            round_vel = -vel_max
        round_vel = int(round_vel) 
        data = [0x0A, 0x0C, 0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, round_vel & 0xFF, (round_vel >> 8) & 0xFF, 0x00]
    else:
        vel = linear
        if vel > vel_max:
            vel = vel_max
        elif vel < -vel_max:
            vel = -vel_max
        if angular != 0:
            R = vel / angular
            if R < 0 and R > -length:
                ang = -ang_max
            elif R > 0 and R < length:
                ang = ang_max
            else:
                ang = math.sin(length / R) * 180 / math.pi
                if ang > ang_max:
                    ang = ang_max
                elif ang < -ang_max:
                    ang = -ang_max
        else:
            ang = 0
        ang = ang * 100
        vel = int(vel)
        ang = int(ang)
        data = [0x0A, 0x0C, 0x06, 0x00, 0x02, vel & 0xFF, (vel >> 8) & 0xFF, ang & 0xFF, (ang >> 8) & 0xFF, 0x00, 0x00, 0x00]
###############################################################################################################################

    # 添加校验位
    data[-1] = check_sum(data)
    # 将整数转换为十六进制字符串
    for i in range(len(data)):
        data[i] = format(data[i], 'X').zfill(2)
    # 转换成字符串数据
    data_str = ''.join(data).encode()
    # 转换成十六进制字节数据
    SerialPort.write(binascii.a2b_hex(data_str))

# 主函数
if __name__ == "__main__":
    # 初始化节点
    rospy.init_node("amc_ros_node", anonymous=True)
    # 获取参数，设置默认值
    serial_name_param = rospy.get_param('~serial_port', 'ttyUSB*')

    # 打印串口名称
    rospy.loginfo(f"Connecting to serial port: {serial_name_param}")
    # 连接串口
    try:
        SerialPort = serial_connect(serial_name_param)
    except Exception as e:
        rospy.logerr(e)
    
    # 订阅速度话题
    rospy.Subscriber('/cmd_vel',geometry_msgs.msg.Twist,handle_data)
    # 使能
    power_on_machine()
    #创建串口接收线程
    t = threading.Thread(target = data_receive_thread)
    t.setDaemon(True)
    t.start()
    while not rospy.is_shutdown():
        time.sleep(1)
    # 失能
    power_off_machine()
