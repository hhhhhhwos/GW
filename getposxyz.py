#!/usr/bin/env python3
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程将订阅/turtle1/pose话题，消息类型turtlesim::Pose

import rospy
import rosbag
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String  # 消息头文件

from livox_ros_driver2.msg import CustomMsg
import numpy as np
import math

str_return1 = ''
str_return2 = ''


def newOdom2(msg):
    global num, str_return1
    # msg = msg.transforms.geometry_msgs.msg._TransformStamped.TransformStamped
    # print((msg.transforms[0].transform.translation.x))
    # print(msg.points[1].x)
    # print(type(msg.points[0]))
    # print(msg.transforms.transform.x)

    # x1 = -msg.pose.position.x
    # y1 = msg.pose.position.y

    # rot_q = msg.pose.orientation
    # (roll, pitch, theta1) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    n = len(msg.points)
    dis0 = []
    dis3 = []
    dis6 = []
    dis9 = []
    for i in range(n):
        X = msg.points[i].x
        Y = msg.points[i].y
        Z = msg.points[i].z
        # print(msg.points[i])

        y2 = Y
        x2 = X * 1.732 / 2 + X * 1 / 2
        z2 = -Z * 1 / 2 + Z * 1.732 / 2

        if X > 0.2 and 0.01 < abs(Y) < 0.2 and 0.01 < abs(Z) < 0.2:
            dis0.append(X)
        if Y > 0.2 and 0.01 < abs(X) < 0.2 and 0.01 < abs(Z) < 0.2:
            dis3.append(Y)
        # if x2 < -0.2 and 0.01 < abs(y2) < 0.2 and 0.01 < abs(z2) < 0.2:
        #     dis6.append(-x2)
        if X < -0.2 and 0.01 < abs(Y) < 0.2 and math.tan(28 / 57.3) < abs(Z / X) < math.tan(32 / 57.3):  # abs(Z/X)*math.tan(28) < Z < abs(Z/X)*math.tan(32)
            dis6.append(-X)
        if Y < -0.2 and 0.01 < abs(X) < 0.2 and 0.01 < abs(Z) < 0.2:
            dis9.append(-Y)

    if len(dis0) <= 2:
        dis0 = 40
    if len(dis3) <= 2:
        dis3 = 40
    if len(dis6) <= 2:
        dis6 = 40
    if len(dis9) <= 2:
        dis9 = 40
    # y = msg.points.y
    # z = msg.points.z
    # print(len(disfront))
    # print(np.mean(disfront))

    # rospy.loginfo("[%0.2f, %0.2f, %0.2f, %0.2f]",np.mean(dis0), np.mean(dis3), np.mean(dis6), np.mean(dis9))
    # print(type(x))

    # x1 = msg.transforms[0].transform.translation.x
    # y1 = msg.transforms[0].transform.translation.y
    # z1 = msg.transforms[0].transform.translation.z

    # rot1 = msg.transforms[0].transform.rotation.x
    # rot2 = msg.transforms[0].transform.rotation.y
    # rot3 = msg.transforms[0].transform.rotation.z
    # rot4 = msg.transforms[0].transform.rotation.w

    # (roll, pitch, theta1) = euler_from_quaternion([rot1, rot2, rot3, rot4])

    str_return1 = "%0.6f  %0.6f  %0.6f" % (np.mean(dis6), np.mean(dis9), np.mean(dis3))
    # print(np.mean(dis6),np.mean(dis9),np.mean(dis3))        #dis6前，dis9左，dis3右
    pub.publish(str_return1)  # 发布字符串
    # print("x0=%0.6f  y0=%0.6f  z0=%0.6f  yaw0=%0.6f" % (x1,-y1,-z1,-theta1))


lis = rospy.init_node('LD06pose_subscriber_2', anonymous=True)
rospy.Subscriber("/livox/lidar", CustomMsg, newOdom2)

# lis2 = rospy.init_node('LD06pose_subscriber_2', anonymous=True)
# rospy.Subscriber("/tf",TFMessage,newOdom1)
pub = rospy.Publisher('chatter_distance0369', String, queue_size=10)  # 发布消息到话题 chatter 中,队列长度10
rate = rospy.Rate(10)
# 循环等待回调函数
rospy.spin()
