#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import serial
from xmodem import XMODEM

import binascii
import numpy as np
import os, signal
import subprocess
import re
import rospy
from std_msgs.msg import String
from pymavlink import mavutil
import roslaunch
import pika
import time
import threading
import xlrd
# from pymavlink import mavutil
# import time
import xlrd2

print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@0')

for i in range(30):
    time.sleep(1)
    print(i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i)

home_lat = 0  # Somewhere random
home_lon = 0  # Somewhere random
home_alt = 0  # Somewhere random

time.sleep(10)

FeiKong_1 = mavutil.mavlink_connection('/dev/Auto', baud=115200)
# FeiKong_1 = mavutil.mavlink_connection('/dev/ttyUSB1', baud=115200)
# ser = serial.Serial('/dev/ttyACM2', baudrate=115200)

date = ('FE 06 00 FF BE 42 02 00 00 00 02 01 1D 88 FE 06 01 FF BE 42 02 00 00 00 02 01 8C DD FE 06 02 FF BE 42 02 '
        '00 00 00 06 01 5E 40 FE 06 03 FF BE 42 02 00 00 00 06 01 CF 15 FE 06 04 FF BE 42 04 00 00 00 0A 01 47 48 '
        'FE 06 05 FF BE 42 04 00 00 00 0A 01 D6 1D FE 06 06 FF BE 42 04 00 00 00 0B 01 B9 B9 FE 06 07 FF BE 42 04 '
        '00 00 00 0B 01 28 EC FE 06 08 FF BE 42 02 00 00 00 0C 01 AC 24 FE 06 09 FF BE 42 02 00 00 00 0C 01 3D 71 '
        'FE 09 0A FF BE 00 00 00 00 00 06 08 00 00 03 9F 50 FE 06 0B FF BE 42 02 00 00 00 01 01 60 25 FE 06 0C FF '
        'BE 42 02 00 00 00 01 01 86 85 FE 06 0D FF BE 42 02 00 00 00 03 01 AF 65 FE 06 0E FF BE 42 02 00 00 00 03 '
        '01 1C 9B')

KG_excel = 0
ctrl_key_2 = 1
t_dianya = 0
V_dianya = 0
rows_date = ''
Mavlink_date = ''
t_1 = ''
t_2 = ''
KG_planoff = 0
KG_land = 0
KG_Auto = 0

LD06_XYZ = [0, 0, 0, 0, 0]
distance0369 = [0, 0, 0]

# time.sleep(10)
My_launch = "cd /home/ubuntu/ubuntu2004+mid360/ws_pointlio2/src/Point-LIO/src;roslaunch mapping_mid360.launch"
os.system("gnome-terminal --geometry=60x1+30+30 -e 'bash -c \"" + My_launch + ";bash\"'")
print('运行Mlaunch')

FeiKong_1.write(bytes.fromhex(date))  # 数据写回
print('on')

user_info = pika.PlainCredentials('nameB', 'passwordB')  # 用户名和密码
connection1 = pika.BlockingConnection(
    pika.ConnectionParameters('192.168.43.251', 5672, '/', user_info))  # 连接服务器上的RabbitMQ服务
connection2 = pika.BlockingConnection(
    pika.ConnectionParameters('192.168.43.251', 5672, '/', user_info))  # 连接服务器上的RabbitMQ服务


def set_rc_channel_pwm(channel_id, pwm=1500):
    rc_channel_values = [65535 for _ in range(9)]
    rc_channel_values[channel_id - 1] = pwm
    FeiKong_1.mav.rc_channels_override_send(
        FeiKong_1.target_system,  # target_system
        FeiKong_1.target_component,  # target_component
        *rc_channel_values)  # RC channel list, in microseconds.


def timer_pwm():
    global timer_t1
    try:
        set_rc_channel_pwm(7, 1450)
        print('set_rc_channel_pwm(7, 1450)')
    except:
        pass
    timer_t1 = threading.Timer(0.2, timer_pwm)
    timer_t1.start()
timer_pwm()


def planoff():
    try:
        set_rc_channel_pwm(5, 1545)
        print('RC="stable"')
        time.sleep(0.5)
        set_rc_channel_pwm(5, 1045)
        print('RC="GUIDED"')
        time.sleep(0.5)
        FeiKong_1.arducopter_arm()
        print("Waiting for the vehicle to arm")
        # FeiKong_1.motors_armed_wait()
        print('Armed')
        FeiKong_1.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                        0, 0, 0, 0, 0, 0, 0, 0.6)
        print('takeoff')
    except:
        pass

    time.sleep(10)

    print('planoff _ over...... Auto  ')


def fun_jiangluo():
    global state_k
    state_k = 5
    for i in range(10):
        # FeiKong_1.wait_heartbeat()
        set_rc_channel_pwm(8, 1945)
        time.sleep(0.5)
        print('RC="LAND"')
    # ser.write('FF 00 03 AA'.encode())  # 数据写回


# 改转速150
def condition_yaw(X, CL):  # +-500
    FeiKong_1.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                                    0, X, 100, CL, 1, 0, 0, 0)
    # 1-角度（10） 2-速率（5） 3-顺时针（0） 4-相对角度 567-空


########################################################################################
flag_photo = 0
Yaw_now = 0


def fun_Auto_callback():
    global home_lon, home_lat, home_alt, Point_fly, distance, max1, xishu, flag_photo
    global altitude, timer_t1
    global frame, switch_return, Yaw_now, state_k
    global k, V_dianya

    switch_return = 0
    state_k = 3

    # print(LD06_XYZ)
    if 0.000001 < abs(LD06_XYZ[0]) < 0.5 and 0.000001 < abs(LD06_XYZ[1]) < 0.5 and 0.000001 < abs(LD06_XYZ[2]) < 0.5:
        print('planoff')
        planoff()
        print('auto')

        file = '/home/ubuntu/ubuntu2004+mid360/ws_pointlio2/src/Point-LIO/src/targetpoints.xls'
        wb = xlrd.open_workbook(filename=file)  # 打开文件
        sheet1 = wb.sheet_by_index(0)  # 通过索引获取表格
        cols = sheet1.col_values(1)  # 获取列内容

        ii = int(len(cols) - 4)
        for i in range(len(cols) - 1):
            if switch_return == 1:
                i = ii
                print(ii)
            rows = sheet1.row_values(i + 1)  # 获取行内容
            if rows[4] == 0:
                # print(rows[1], rows[2], rows[3])
                X1 = rows[1]
                Y1 = rows[2]
                altitude = rows[3]

                print(altitude)
                XiShu = 40000 / 360 * 1000
                home_lat1 = home_lat + X1 / XiShu
                home_lon1 = home_lon + Y1 / XiShu
                # 发送指令，让无人机前往第一个航点
                if state_k != 8:
                    print("Fly to ", i + 1, "Point", rows)
                    FeiKong_1.mav.mission_item_send(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                                0, 0, 0, home_lat1, home_lon1,
                                                altitude)
            else:
                if state_k != 8:
                    condition_yaw(int(abs(rows[4])), int(np.sign(rows[4])))
                    Yaw_now = Yaw_now + rows[4]
            waittime = rows[0]
            if waittime <= 5:
                waittime = 5
            time.sleep(waittime)

            if switch_return == 1:
                if ii == int(len(cols) - 4):
                    # time.sleep(6)  # 返回等待时间
                    condition_yaw(int(abs(Yaw_now)), -int(np.sign(Yaw_now)))
                    time.sleep(12)
                ii = ii + 1
                if ii == len(cols) - 1:
                    break

            msg_str = '{k=%0.3f,Power:%0.3f,Xyz:%0.3f,%0.3f,%0.3f,flag_photo:1}}' % (
                k, V_dianya, LD06_XYZ[0], LD06_XYZ[1], LD06_XYZ[2])
            channel1.basic_publish(exchange='',  # 当前是一个简单模式，所以这里设置为空字符串就可以了
                                   routing_key='plane_STATUS_QUEUE',  # 指定消息要发送到哪个queue
                                   # body='{}'.format(i)  # 指定要发送的消息
                                   body=msg_str,
                                   properties=pika.BasicProperties(delivery_mode=2)
                                   )
        print('Land')
        if state_k != 8:
            fun_jiangluo()
    else:
        pass
    print('over!!!!')

    if state_k != 8:
        msg_str = '{k=%0.3f,clientNo:3,planeStatus:0%0.0f,Power:%0.3f,Xyz:%0.3f,%0.3f,%0.3f,flag_photo:0}}' % (
            0, 7, V_dianya, LD06_XYZ[0], LD06_XYZ[1], LD06_XYZ[2])
        channel1.basic_publish(exchange='',  # 当前是一个简单模式，所以这里设置为空字符串就可以了
                               routing_key='plane_STATUS_QUEUE',  # 指定消息要发送到哪个queue
                               # body='{}'.format(i)  # 指定要发送的消息
                               body=msg_str,
                               properties=pika.BasicProperties(delivery_mode=2)
                               )

        state_k = 1


switch_return = 0


def fun_return():
    global switch_return, state_k
    switch_return = 1
    state_k = 4
    # pass


#####################################   接收控制指令   #################################################
def callback(ch, method, properties, body):
    # print('planemsg:{}'.format(body))
    # print(type(body))
    body_str = str(body, 'utf-8')
    print(type(body_str))

    if body_str == '{"clientNo":3,"commandNo":03}':
        print('Auto ok.')
        threading.Timer(0.1, fun_Auto_callback).start()
        # fun_Auto_callback()
        print('Auto ok.')
    if body_str == '{"clientNo":3,"commandNo":05}':
        print('return ok.')
        fun_return()
        print('return ok.')
    if body_str == '{"clientNo":3,"commandNo":07}':
        print('land ok.')
        fun_jiangluo()
        print('land ok.')


def num_temp():
    print(00000)
    channel2 = connection2.channel()
    channel2.queue_declare(queue='plane_COMMAND_QUEUE', durable=True)
    channel2.queue_purge('plane_COMMAND_QUEUE')
    channel2.basic_consume(queue='plane_COMMAND_QUEUE',  # 接收指定queue的消息
                           auto_ack=True,  # 指定为True，表示消息接收到后自动给消息发送方回复确认，已收到消息
                           on_message_callback=callback  # 设置收到消息的回调函数
                           )
    channel2.start_consuming()


def timer_fun():
    threading.Timer(0.1, num_temp).start()
    print(111111111111111111)


timer_fun()

#####################################   send制指令   #################################################
# 创建一个channel
channel1 = connection1.channel()
# 如果指定的queue不存在，则会创建一个queue，如果已经存在 则不会做其他动作，官方推荐，每次使用时都可以加上这句
channel1.queue_declare(queue='plane_STATUS_QUEUE', durable=True)
k = 1
state_k = 1


def fun_sendmsg():
    global k, X_mav, Y_mav, Yaw_mav, Z_mav, V_dianya, state_k
    # msg_str = '{k=%0.3f,clientNo:3,planeStatus:09,data:{planePower:55.6,Xyz:%0.3f,%0.3f,%0.3f,flag_photo:%0.0f}}'%(k,X_mav,Y_mav,Z_mav,flag_photo)
    msg_str = '{k=%0.3f,clientNo:3,planeStatus:0%0.0f,Power:%0.3f,Xyz:%0.3f,%0.3f,%0.3f,flag_photo:0}}' % (
        k, state_k, V_dianya, LD06_XYZ[0], LD06_XYZ[1], LD06_XYZ[2])
    # print(msg_str)
    channel1.basic_publish(exchange='',  # 当前是一个简单模式，所以这里设置为空字符串就可以了
                           routing_key='plane_STATUS_QUEUE',  # 指定消息要发送到哪个queue
                           # body='{}'.format(i)  # 指定要发送的消息
                           body=msg_str,
                           properties=pika.BasicProperties(delivery_mode=2)
                           )
    # print(msg_str)
    # flag_photo = 0
    k = k + 0.1
    threading.Timer(0.3, fun_sendmsg).start()


fun_sendmsg()

ctrl_key_1 = 0


def fun_YaoKong():
    # 通过遥控器控制巡线飞行
    global ctrl_key_1, My_Cmmnd, state_k
    # FeiKong.wait_heartbeat()
    try:
        message1 = FeiKong_1.recv_match(type='RC_CHANNELS', blocking=True).to_dict()
        RC9 = round(float(message1['chan9_raw']), 2)
        if RC9 > 1700 and ctrl_key_1 == 1:
            print('开启自动避障模式')
            # if np.min(distance0369) < 2:
            if distance0369[0] < 2:
                set_rc_channel_pwm(5, 1495)
                time.sleep(0.5)
                state_k = 8
                print('RC="Position Hold"')
                ctrl_key_1 = 0

        if RC9 <= 1700 and ctrl_key_1 == 0:
            print('关闭自动避障模式')
            ctrl_key_1 = 1
            # fun_SetHome()
        # print(RC9)
    except:
        pass
        # print('getmcu RC9 error')
    threading.Timer(0.2, fun_YaoKong).start()

print('test1')

fun_YaoKong()

print('test2')

channel1.start_consuming()

print('test3')


def fun_getxyz_2(msg):
    num = re.findall(r"-?\d+\.?\d*", msg.data)
    # print(msg.data)
    if len(num) >= 5:
        x1 = float(num[0])
        y1 = float(num[1])
        z1 = float(num[2])
        Yaw1 = float(num[3])

        y2 = y1
        x2 = x1 * 1.732 / 2 + z1 * 1 / 2
        z2 = -x1 * 1 / 2 + z1 * 1.732 / 2
        Yaw2 = Yaw1 + 15 * np.sin(Yaw1) / 57.3

        LD06_XYZ[0] = -x2
        LD06_XYZ[1] = -y2
        LD06_XYZ[2] = z2
        LD06_XYZ[3] = Yaw2
        LD06_XYZ[4] = float(num[4])

    # print(LD06_XYZ)
    # print('11111111111')


def fun_getxyz0369(msg):
    num = re.findall(r"-?\d+\.?\d*", msg.data)
    # print(msg.data)
    if len(num) >= 3:
        distance0369[0] = float(num[0])
        distance0369[1] = float(num[1])
        distance0369[2] = float(num[2])

    # print(distance0369)
    # print('11111111111')


print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@4')

lis = rospy.init_node('Main_subscriber_2', anonymous=True)
# 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
rospy.Subscriber("/chatter_LD360", String, fun_getxyz_2)
# rospy.Subscriber("/chatter_ego", String, fun_getxyz_ego)
rospy.Subscriber("/chatter_distance0369", String, fun_getxyz0369)

rospy.spin()

print('000000000000000000000000000')
