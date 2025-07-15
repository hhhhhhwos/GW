#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ==============================================================================
# 无人机自动飞行控制系统
# 功能：通过MAVLink协议控制无人机，支持自动航线飞行、避障、遥控等功能
# ==============================================================================

# 导入必要的库
import numpy as np              # 数值计算库
import serial                   # 串口通信库
from xmodem import XMODEM       # XMODEM协议库

import binascii                 # 二进制和ASCII转换
import numpy as np              # 数值计算
import os, signal               # 操作系统接口
import subprocess               # 子进程管理
import re                       # 正则表达式
import rospy                    # ROS Python客户端库
from std_msgs.msg import String # ROS标准消息类型
from pymavlink import mavutil   # MAVLink协议库，用于与飞控通信
import roslaunch                # ROS启动文件管理
import pika                     # RabbitMQ Python客户端
import time                     # 时间相关函数
import threading                # 多线程库
import xlrd                     # Excel文件读取库（老版本）
import xlrd2                    # Excel文件读取库（新版本）

# 启动提示信息
print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@0')

# 初始化等待30秒，打印倒计时
for i in range(30):
    time.sleep(1)
    print(i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i)

# ==============================================================================
# 全局变量定义
# ==============================================================================

# HOME点坐标定义（起飞点/返航点）
home_lat = 0  # HOME点纬度 - 某个随机位置
home_lon = 0  # HOME点经度 - 某个随机位置  
home_alt = 0  # HOME点海拔 - 某个随机位置

time.sleep(10)  # 等待10秒

# ==============================================================================
# 飞控连接初始化
# ==============================================================================

# 建立与飞控的MAVLink连接
FeiKong_1 = mavutil.mavlink_connection('/dev/Auto', baud=115200)
# 备用连接方式：
# FeiKong_1 = mavutil.mavlink_connection('/dev/ttyUSB1', baud=115200)
# ser = serial.Serial('/dev/ttyACM2', baudrate=115200)

# 预定义的十六进制数据包（可能是初始化配置数据）
date = ('FE 06 00 FF BE 42 02 00 00 00 02 01 1D 88 FE 06 01 FF BE 42 02 00 00 00 02 01 8C DD FE 06 02 FF BE 42 02 '
        '00 00 00 06 01 5E 40 FE 06 03 FF BE 42 02 00 00 00 06 01 CF 15 FE 06 04 FF BE 42 04 00 00 00 0A 01 47 48 '
        'FE 06 05 FF BE 42 04 00 00 00 0A 01 D6 1D FE 06 06 FF BE 42 04 00 00 00 0B 01 B9 B9 FE 06 07 FF BE 42 04 '
        '00 00 00 0B 01 28 EC FE 06 08 FF BE 42 02 00 00 00 0C 01 AC 24 FE 06 09 FF BE 42 02 00 00 00 0C 01 3D 71 '
        'FE 09 0A FF BE 00 00 00 00 00 06 08 00 00 03 9F 50 FE 06 0B FF BE 42 02 00 00 00 01 01 60 25 FE 06 0C FF '
        'BE 42 02 00 00 00 01 01 86 85 FE 06 0D FF BE 42 02 00 00 00 03 01 AF 65 FE 06 0E FF BE 42 02 00 00 00 03 '
        '01 1C 9B')

# 控制变量和状态变量定义
KG_excel = 0          # Excel控制开关
ctrl_key_2 = 1        # 控制键2
t_dianya = 0          # 电压时间
V_dianya = 0          # 电压值
rows_date = ''        # 行数据
Mavlink_date = ''     # MAVLink数据
t_1 = ''              # 时间1
t_2 = ''              # 时间2
KG_planoff = 0        # 起飞开关
KG_land = 0           # 降落开关
KG_Auto = 0           # 自动模式开关

# 激光雷达位置数据：[X, Y, Z, Yaw, 额外参数]
LD06_XYZ = [0, 0, 0, 0, 0]
# 距离传感器数据：[0度, 3度, 6度, 9度方向的距离]
distance0369 = [0, 0, 0]
jiangluo_flag = 0     # 降落标志

# ==============================================================================
# 系统初始化
# ==============================================================================

# 启动激光雷达映射服务
My_launch = "cd /home/ubuntu/ubuntu2004+mid360/ws_pointlio2/src/Point-LIO/src;roslaunch mapping_mid360.launch"
os.system("gnome-terminal --geometry=60x1+30+30 -e 'bash -c \"" + My_launch + ";bash\"'")
print('运行Mlaunch')  # 启动映射launch文件

# 向飞控发送初始化数据
FeiKong_1.write(bytes.fromhex(date))
print('on')  # 发送初始化命令

# ==============================================================================
# RabbitMQ连接初始化
# ==============================================================================

# 设置RabbitMQ连接参数
user_info = pika.PlainCredentials('nameB', 'passwordB')  # 用户名和密码
# 建立两个独立的连接（一个用于发送，一个用于接收）
connection1 = pika.BlockingConnection(
    pika.ConnectionParameters('192.168.31.36', 5672, '/', user_info))  # 连接服务器上的RabbitMQ服务
connection2 = pika.BlockingConnection(
    pika.ConnectionParameters('192.168.31.36', 5672, '/', user_info))  # 连接服务器上的RabbitMQ服务

# ==============================================================================
# 飞控控制函数定义
# ==============================================================================

def set_rc_channel_pwm(channel_id, pwm=1500):
    """
    设置遥控通道的PWM值
    
    参数:
        channel_id: 通道编号（1-9）
        pwm: PWM值（微秒），通常范围1000-2000，1500为中位
    """
    # 初始化所有通道为65535（无效值）
    rc_channel_values = [65535 for _ in range(9)]
    # 设置指定通道的PWM值
    rc_channel_values[channel_id - 1] = pwm
    # 发送遥控器覆盖命令
    FeiKong_1.mav.rc_channels_override_send(
        FeiKong_1.target_system,    # 目标系统ID
        FeiKong_1.target_component, # 目标组件ID
        *rc_channel_values)         # RC通道列表，单位微秒

def timer_pwm():
    """
    定时器函数：定期发送PWM信号
    用于保持某些控制通道的状态
    """
    global timer_t1
    try:
        set_rc_channel_pwm(7, 1050)  # 设置通道7为1050微秒
    except:
        pass
    # 设置0.2秒后再次执行
    timer_t1 = threading.Timer(0.2, timer_pwm)
    timer_t1.start()

def planoff():
    """
    无人机起飞程序
    执行顺序：设置HOME点 -> 切换到GUIDED模式 -> 解锁 -> 起飞
    """
    try:
        # 1. 设置HOME点（重复5次确保成功）
        for i in range(5):
            fun_SetHome()
            time.sleep(0.1)
        
        # 2. 切换到GUIDED模式（重复5次确保成功）
        for i in range(5):
            set_rc_channel_pwm(5, 1045)  # 通道5控制飞行模式
            time.sleep(0.1)
            print('RC="GUIDED"')
        
        # 3. 解锁电机
        FeiKong_1.arducopter_arm()
        print("Waiting for the vehicle to arm")
        # FeiKong_1.motors_armed_wait()  # 等待电机解锁完成（已注释）
        print('Armed')
        
        # 4. 发送起飞命令，目标高度0.7米
        FeiKong_1.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                        0, 0, 0, 0, 0, 0, 0, 0.7)
        print('takeoff')
    except:
        pass

    time.sleep(10)  # 等待起飞完成
    print('planoff _ over...... Auto  ')

def fun_jiangluo():
    """
    无人机降落程序
    切换到LAND模式并等待降落完成，然后上锁电机
    """
    global state_k
    state_k = 5  # 设置状态为降落模式
    
    # 连续发送降落命令（100次确保成功）
    for i in range(100):
        # FeiKong_1.wait_heartbeat()  # 等待心跳（已注释）
        set_rc_channel_pwm(8, 1945)  # 通道8控制降落
        time.sleep(0.1)
        print('RC="LAND"')
    
    # 降落完成后锁定电机
    FeiKong_1.arducopter_disarm()
    state_k = 1  # 重置状态

def condition_yaw(X, CL):
    """
    控制无人机偏航角度
    
    参数:
        X: 目标角度
        CL: 方向控制（正数顺时针，负数逆时针）
    """
    # 发送偏航条件命令
    FeiKong_1.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                                    0, X, 100, CL, 1, 0, 0, 0)
    # 参数说明：
    # 1-角度（X） 2-速率（100） 3-方向（CL） 4-相对角度标志 5,6,7-保留

# ==============================================================================
# 自动飞行控制函数
# ==============================================================================

# 飞行状态和控制变量
flag_photo = 0        # 拍照标志
Yaw_now = 0          # 当前偏航角
fanhang_kg = 0       # 返航开关

def fun_Auto_callback():
    """
    自动飞行主控制函数
    读取Excel文件中的航点信息，控制无人机按航线飞行
    """
    global home_lon, home_lat, home_alt, Point_fly, distance, max1, xishu, flag_photo
    global altitude, timer_t1, jishu
    global frame, switch_return, Yaw_now, state_k
    global k, V_dianya, jiangluo_flag, airline_kg, airline_cont, content
    global X1, Y1, angle_do, temperature, humidity, tongji_NO, fanhang_kg, photo_kg, bizhang_kg

    # 初始化飞行参数
    Yaw_now = 0           # 当前偏航角
    jiangluo_flag = 0     # 降落标志
    switch_return = 0     # 返航开关
    state_k = 3           # 设置为自动飞行状态
    i = 0                 # 航点索引
    data_k = 3            # 数据控制变量

    # 检查激光雷达位置数据是否有效
    # 确保X、Y、Z坐标都在有效范围内（0.000001到0.5米之间）
    if 0.000001 < abs(LD06_XYZ[0]) < 0.5 and 0.000001 < abs(LD06_XYZ[1]) < 0.5 and 0.000001 < abs(LD06_XYZ[2]) < 0.5:
        print('planoff')
        planoff()  # 执行起飞程序
        print('auto')

        # 读取航点文件
        file = '/home/ubuntu/ubuntu2004+mid360/ws_pointlio2/src/Point-LIO/src/targetpoints.xls'
        wb = xlrd.open_workbook(filename=file)  # 打开Excel文件
        sheet1 = wb.sheet_by_index(0)           # 获取第一个工作表
        cols = sheet1.col_values(1)             # 获取第二列的所有值

        # 主飞行循环
        while i < int(len(cols)) - 1:
            # 根据返航标志决定飞行方向
            if switch_return == 0:
                # 正常前进模式
                i = i + 1
            elif switch_return == 1:
                # 返航模式：根据当前位置决定前进还是后退
                if i >= int(len(cols)) - round(int(len(cols)) / 4):
                    i = i + 1  # 在航线后1/4段时继续前进
                else:
                    i = i - 1  # 在航线前3/4段时后退
                    if i <= 0 or data_k <= 2:
                        i = int(len(cols)) - data_k + 1
                        data_k = data_k - 1
            
            # 检查是否需要强制降落
            if jiangluo_flag == 1:
                break

            # 读取当前航点数据
            rows = sheet1.row_values(i)  # 获取当前行的所有数据
            # Excel列结构：[等待时间, X坐标, Y坐标, 高度, 偏航角, ...]

            # 计算偏航角差值
            angle_do = rows[4] - Yaw_now    # 目标偏航角 - 当前偏航角
            angle_do2 = angle_do
            
            # 角度归一化到[-180, 180]范围内
            if angle_do >= 180:
                angle_do2 = -360 + angle_do
            if angle_do < -180:
                angle_do2 = 360 + angle_do

            # 如果需要改变偏航角且不在避障状态
            if Yaw_now != rows[4] and state_k != 8:
                condition_yaw(int(abs(angle_do2)), int(np.sign(angle_do2)))  # 执行偏航
                Yaw_now = rows[4]  # 更新当前偏航角
                print(Yaw_now, Yaw_now, Yaw_now, Yaw_now, Yaw_now, Yaw_now, Yaw_now)
                # 根据角度大小计算等待时间（60度/秒的转速）
                time.sleep(int((abs(angle_do2) / 60 * 3.5)))

            # 获取目标位置坐标
            X1 = rows[1]        # 目标X坐标
            Y1 = rows[2]        # 目标Y坐标  
            altitude = rows[3]  # 目标高度

            print(altitude)
            
            # 坐标转换：将本地坐标转换为GPS坐标
            XiShu = 40000 / 360 * 1000  # 地球周长转换系数
            home_lat1 = home_lat + X1 / XiShu  # 计算目标纬度
            home_lon1 = home_lon + Y1 / XiShu  # 计算目标经度

            # 发送航点命令（如果不在避障状态）
            if state_k != 8 and bizhang_kg != 2:
                print("Fly to ", i, "Point", rows)
                # 发送任务航点命令
                FeiKong_1.mav.mission_item_send(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                                0, 0, 0, home_lat1, home_lon1,
                                                altitude)

                # 检查是否到达目标位置（误差容限0.5米）
                waittimeflag = abs(LD06_XYZ[0] - X1) < 0.5 and abs(LD06_XYZ[1] - Y1) < 0.5 and abs(
                    abs(LD06_XYZ[2]) - abs(altitude)) < 0.5
                
                print('等待4秒')
                time.sleep(4)

                # 如果未到达目标位置，持续发送航点命令
                while waittimeflag == 0 and fanhang_kg == 0 and state_k != 8:
                    FeiKong_1.mav.mission_item_send(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                                    0, 0, 0, home_lat1, home_lon1,
                                                    altitude)
                    print('等待8秒')
                    time.sleep(4)

                    # 重新检查是否到达目标位置
                    waittimeflag = abs(LD06_XYZ[0] - X1) < 0.5 and abs(LD06_XYZ[1] - Y1) < 0.5 and abs(
                        abs(LD06_XYZ[2]) - abs(altitude)) < 0.5

                print('等待12秒')
                time.sleep(4)

            # 避障状态处理
            while state_k == 8 and bizhang_kg == 0:
                print('触发避障。。。。。。触发避障。。。。。。触发避障')

            # 避障恢复处理
            if bizhang_kg == 1:
                set_rc_channel_pwm(5, 1045)  # 切换到GUIDED模式
                time.sleep(0.5)

                # 重新检查位置
                waittimeflag = abs(LD06_XYZ[0] - X1) < 0.5 and abs(LD06_XYZ[1] - Y1) < 0.5 and abs(
                    abs(LD06_XYZ[2]) - abs(altitude)) < 0.5

                # 继续飞向目标点
                while waittimeflag == 0 and fanhang_kg == 0 and state_k != 8:
                    FeiKong_1.mav.mission_item_send(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                                    0, 0, 0, home_lat1, home_lon1,
                                                    altitude)
                    time.sleep(2)

                    waittimeflag = abs(LD06_XYZ[0] - X1) < 0.5 and abs(LD06_XYZ[1] - Y1) < 0.5 and abs(
                        abs(LD06_XYZ[2]) - abs(altitude)) < 0.5
                bizhang_kg = 0  # 重置避障标志

            # 强制退出避障
            if bizhang_kg == 2:
                print('break out!!!!')
                break

            # 发送状态消息到RabbitMQ
            msg_str = '{k=%0.3f,Power:%0.3f,Xyz:%0.3f,%0.3f,%0.3f,%0.3f,flag_photo:1}}' % (
                k, V_dianya, LD06_XYZ[0], LD06_XYZ[1], LD06_XYZ[2], LD06_XYZ[3])
            channel1.basic_publish(exchange='',
                                   routing_key='plane_STATUS_QUEUE',
                                   body=msg_str,
                                   properties=pika.BasicProperties(delivery_mode=2)
                                   )
        
        # 航线飞行完成，执行降落
        print('Land')
        fun_jiangluo()
    else:
        pass
    
    print('over!!!!')

    # 发送任务完成状态消息
    msg_str = '{k=%0.3f,clientNo:3,planeStatus:0%0.0f,Power:%0.3f,Xyz:%0.3f,%0.3f,%0.3f,%0.3f,flag_photo:0}}' % (
        0, 7, V_dianya, LD06_XYZ[0], LD06_XYZ[1], LD06_XYZ[2], LD06_XYZ[3])
    channel1.basic_publish(exchange='',
                           routing_key='plane_STATUS_QUEUE',
                           body=msg_str,
                           properties=pika.BasicProperties(delivery_mode=2)
                           )

    state_k = 1  # 重置状态

# 返航控制变量
switch_return = 0

def fun_return():
    """
    返航函数
    设置返航标志，让无人机返回起点
    """
    global switch_return, state_k, fanhang_kg
    switch_return = 1  # 启动返航模式
    fanhang_kg = 1     # 设置返航标志
    state_k = 4        # 设置为返航状态

# ==============================================================================
# RabbitMQ消息处理
# ==============================================================================

def callback(ch, method, properties, body):
    """
    RabbitMQ消息回调函数
    处理从地面站发来的控制命令
    """
    global jiangluo_flag
    # print('planemsg:{}'.format(body))
    # print(type(body))
    body_str = str(body, 'utf-8')  # 将字节转换为字符串
    print(type(body_str))

    # 命令解析和执行
    if body_str == '{"clientNo":3,"commandNo":03}':
        # 自动飞行命令
        print('Auto ok.')
        threading.Timer(0.1, fun_Auto_callback).start()  # 启动自动飞行
        print('Auto ok.')
    
    if body_str == '{"clientNo":3,"commandNo":05}':
        # 返航命令
        print('return ok.')
        fun_return()  # 执行返航
        print('return ok.')
    
    if body_str == '{"clientNo":3,"commandNo":07}':
        # 降落命令
        print('land ok.')
        jiangluo_flag = 1  # 设置降落标志
        print('land ok.')

def num_temp():
    """
    消息接收线程函数
    持续监听RabbitMQ队列中的控制命令
    """
    print(00000)
    channel2 = connection2.channel()
    channel2.queue_declare(queue='plane_COMMAND_QUEUE', durable=True)  # 声明命令队列
    channel2.queue_purge('plane_COMMAND_QUEUE')  # 清空队列
    channel2.basic_consume(queue='plane_COMMAND_QUEUE',
                           auto_ack=True,
                           on_message_callback=callback)  # 设置消息回调
    channel2.start_consuming()  # 开始消费消息

def timer_fun():
    """
    启动消息接收线程
    """
    threading.Timer(0.1, num_temp).start()
    print(111111111111111111)

# 启动消息接收
timer_fun()

# ==============================================================================
# 状态发送和电压监控
# ==============================================================================

# 创建状态发送通道
channel1 = connection1.channel()
channel1.queue_declare(queue='plane_STATUS_QUEUE', durable=True)  # 声明状态队列
k = 1        # 消息计数器
state_k = 1  # 系统状态

def fun_sendmsg():
    """
    定时发送无人机状态消息
    包括位置、电压、飞行状态等信息
    """
    global k, X_mav, Y_mav, Yaw_mav, Z_mav, V_dianya, state_k, switch_return, fanhang_kg
    
    if V_dianya > 16.5:
        # 电压正常，发送正常状态
        msg_str = '{k=%0.3f,clientNo:3,planeStatus:0%0.0f,Power:%0.3f,Xyz:%0.3f,%0.3f,%0.3f,%0.3f,flag_photo:0}}' % (
            k, state_k, V_dianya, LD06_XYZ[0], LD06_XYZ[1], LD06_XYZ[2], LD06_XYZ[3])
        channel1.basic_publish(exchange='',
                               routing_key='plane_STATUS_QUEUE',
                               body=msg_str,
                               properties=pika.BasicProperties(delivery_mode=2))
    elif 0 < V_dianya <= 16.5:
        # 电压过低，自动返航
        print('电压过低，自动返航')
        switch_return = 1  # 启动返航
        state_k = 4        # 设置返航状态
        fanhang_kg = 1     # 设置返航标志
        msg_str = '{k=%0.3f,clientNo:3,planeStatus:0%0.0f,Power:%0.3f,Xyz:%0.3f,%0.3f,%0.3f,%0.3f,flag_photo:0}}' % (
            0, 4, V_dianya, LD06_XYZ[0], LD06_XYZ[1], LD06_XYZ[2], LD06_XYZ[3])
        channel1.basic_publish(exchange='',
                               routing_key='plane_STATUS_QUEUE',
                               body=msg_str,
                               properties=pika.BasicProperties(delivery_mode=2))
    
    k = k + 0.1  # 更新消息计数器
    threading.Timer(0.3, fun_sendmsg).start()  # 每0.3秒发送一次

# 启动状态发送
fun_sendmsg()

# ==============================================================================
# 遥控器监控和避障系统
# ==============================================================================

# 遥控和避障控制变量
ctrl_key_0 = 0      # 返航控制键状态
ctrl_key_1 = 0      # 避障控制键状态
bizhang_jishu = 0   # 避障计数器
bizhang_kg = 0      # 避障开关

def fun_YaoKong():
    """
    遥控器监控函数
    监控遥控器通道，处理手动控制和避障逻辑
    """
    global ctrl_key_1, My_Cmmnd, state_k, bizhang_jishu, bizhang_kg, V_dianya, ctrl_key_0
    
    try:
        # 读取遥控器通道数据
        message1 = FeiKong_1.recv_match(type='RC_CHANNELS', blocking=True).to_dict()
        # 读取电池状态
        message = FeiKong_1.recv_match(type='BATTERY_STATUS', blocking=True).to_dict()
        V_dianya = message['voltages'][0] / 1000  # 电压值（伏特）
        
        # 读取遥控器通道值
        RC7 = round(float(message1['chan7_raw']), 2)  # 通道7（返航开关）
        RC9 = round(float(message1['chan9_raw']), 2)  # 通道9（避障开关）
        
        # 返航控制逻辑
        if RC7 > 1700 and ctrl_key_0 == 1:
            print('开启返航模式')
            fun_return()
            ctrl_key_0 = 0

        if RC7 <= 1700 and ctrl_key_0 == 0:
            print('关闭返航模式')
            ctrl_key_0 = 1

        # 避障控制逻辑
        if RC9 > 1700 and ctrl_key_1 == 1:
            bizhang_jishu = bizhang_jishu + 1
            print('开启自动避障模式')
            
            # 检查前方距离是否小于2.5米
            if distance0369[0] < 2.5:
                set_rc_channel_pwm(5, 1495)  # 切换到位置保持模式
                state_k = 8  # 设置为避障状态
                print('RC="Position Hold"')
            
            # 避障计数达到50次时的处理
            if bizhang_jishu >= 50:
                if distance0369[0] >= 2.5:
                    # 距离安全，恢复自动飞行
                    state_k = 3
                    bizhang_kg = 1
                elif distance0369[0] < 2.5:
                    # 距离仍然危险，强制降落
                    state_k = 8
                    bizhang_kg = 2
                    print('Land,distance<2.5m 10s')
                    fun_jiangluo()
                
                print(bizhang_kg, bizhang_kg, bizhang_kg, bizhang_kg, bizhang_kg, bizhang_kg, bizhang_kg)
                bizhang_jishu = 0  # 重置计数器
                ctrl_key_1 = 0

        if RC9 <= 1700 and ctrl_key_1 == 0:
            print('关闭自动避障模式')
            ctrl_key_1 = 1
            bizhang_jishu = 0
    except:
        pass
        # print('getmcu RC9 error')  # 获取遥控器数据出错
    
    # 每0.2秒执行一次
    threading.Timer(0.2, fun_YaoKong).start()

# 启动遥控器监控
fun_YaoKong()

# 开始消费状态消息队列
channel1.start_consuming()

# ==============================================================================
# ROS节点和回调函数
# ==============================================================================

def fun_getxyz_2(msg):
    """
    激光雷达位置数据回调函数
    接收来自Point-LIO的位置和姿态信息
    """
    # 从消息中提取数字
    num = re.findall(r"-?\d+\.?\d*", msg.data)
    
    if len(num) >= 5:
        # 原始数据
        x1 = float(num[0])   # X坐标
        y1 = float(num[1])   # Y坐标  
        z1 = float(num[2])   # Z坐标
        Yaw1 = float(num[3]) # 偏航角
        
        # 坐标系转换（旋转变换）
        y2 = y1
        x2 = x1 * 1.732 / 2 + z1 * 1 / 2      # 30度旋转变换
        z2 = -x1 * 1 / 2 + z1 * 1.732 / 2     # 30度旋转变换
        Yaw2 = Yaw1 + 15 * np.sin(Yaw1) / 57.3  # 偏航角修正
        
        # 更新全局位置数据（取负值，可能是坐标系方向调整）
        LD06_XYZ[0] = -x2    # X坐标
        LD06_XYZ[1] = -y2    # Y坐标
        LD06_XYZ[2] = z2     # Z坐标
        LD06_XYZ[3] = Yaw2   # 偏航角
        LD06_XYZ[4] = float(num[4])  # 额外参数

def fun_getxyz0369(msg):
    """
    距离传感器数据回调函数
    接收多个方向的距离测量值，用于避障
    """
    # 从消息中提取数字
    num = re.findall(r"-?\d+\.?\d*", msg.data)
    
    if len(num) >= 3:
        distance0369[0] = float(num[0])  # 0度方向距离
        distance0369[1] = float(num[1])  # 3度方向距离
        distance0369[2] = float(num[2])  # 6度方向距离

# ==============================================================================
# HOME点设置函数
# ==============================================================================

def fun_SetHome():
    """
    设置HOME点（起飞点/返航点）
    """
    set_default_home_position()   # 设置默认HOME位置
    set_default_global_origin()   # 设置全局原点
    print('Home position')

def set_default_global_origin():
    """
    设置GPS全局原点
    """
    FeiKong_1.mav.set_gps_global_origin_send(
        1,          # target_system
        home_lat,   # 纬度
        home_lon,   # 经度  
        home_alt    # 海拔
    )

def set_default_home_position():
    """
    设置HOME位置参数
    """
    x = 0  # 本地X坐标
    y = 0  # 本地Y坐标
    z = 0  # 本地Z坐标
    q = [1, 0, 0, 0]  # 四元数 (w, x, y, z)
    approach_x = 0    # 进近方向X
    approach_y = 0    # 进近方向Y
    approach_z = 1    # 进近方向Z
    
    FeiKong_1.mav.set_home_position_send(1, home_lat, home_lon, home_alt, 
                                         x, y, z, q, 
                                         approach_x, approach_y, approach_z)

# ==============================================================================
# 系统启动和初始化
# ==============================================================================

print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@3')

# 尝试设置HOME点（重复10次确保成功）
for i in range(10):
    try:
        print('sethome = %0.1f"' % (i))
        fun_SetHome()
        time.sleep(1)
    except:
        print('sethome errror %0.1f"' % (i))

print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@4')

# ==============================================================================
# ROS节点初始化和话题订阅
# ==============================================================================

# 初始化ROS节点
lis = rospy.init_node('Main_subscriber_2', anonymous=True)

# 订阅ROS话题
rospy.Subscriber("/chatter_LD360", String, fun_getxyz_2)        # 订阅激光雷达位置话题
rospy.Subscriber("/chatter_distance0369", String, fun_getxyz0369)  # 订阅距离传感器话题

# 保持ROS节点运行
rospy.spin()

print('000000000000000000000000000')