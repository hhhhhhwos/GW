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
#from sensor_msgs.msg import Imu
from std_msgs.msg import String  # 消息头文件
from nav_msgs.msg import Odometry

str_return1 = ''
str_return2 = ''


# def newOdom1(msg):
#     global num, str_return1
#     #msg = msg.transforms.geometry_msgs.msg._TransformStamped.TransformStamped
#     #print((msg.transforms[0].transform.translation.x))
#     #print(msg.transforms)
#     #print(msg.transforms.transform.x)

#     #x1 = -msg.pose.position.x
#     #y1 = msg.pose.position.y

#     #rot_q = msg.pose.orientation
#     #(roll, pitch, theta1) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

#     # rospy.loginfo("pose_LD06: x:%0.2f, y:%0.2f, Yaw:%0.2f", x, y,theta)
#     # print(type(x))

#     x1 = msg.transforms[0].transform.translation.x
#     y1 = msg.transforms[0].transform.translation.y
#     z1 = msg.transforms[0].transform.translation.z
    
#     rot1 = msg.transforms[0].transform.rotation.x
#     rot2 = msg.transforms[0].transform.rotation.y
#     rot3 = msg.transforms[0].transform.rotation.z
#     rot4 = msg.transforms[0].transform.rotation.w
     
#     (roll1, pitch1, theta1) = euler_from_quaternion([rot1, rot2, rot3, rot4])

#     str_return1 = "%0.6f  %0.6f  %0.6f  %0.6f  %0.6f" % (x1,-y1,-z1,-theta1,roll1)

#     pub.publish(str_return1)  # 发布字符串



    
    #print("x0=%0.6f  y0=%0.6f  z0=%0.6f  yaw0=%0.6f" % (x1,-y1,-z1,-theta1))

def cb_save_cur_odom(odom_msg):
    x2 = odom_msg.pose.pose.position.x
    y2 = odom_msg.pose.pose.position.y
    z2 = odom_msg.pose.pose.position.z
    
    rot1_2 = odom_msg.pose.pose.orientation.x
    rot2_2 = odom_msg.pose.pose.orientation.y
    rot3_2 = odom_msg.pose.pose.orientation.z
    rot4_2 = odom_msg.pose.pose.orientation.w
     
    (roll2, pitch2, theta2) = euler_from_quaternion([rot1_2, rot2_2, rot3_2, rot4_2])

    str_return2 = "%0.6f  %0.6f  %0.6f  %0.6f  %0.6f" % (x2,-y2,-z2,-theta2,roll2)
    
    pub.publish(str_return2)  # 发布字符串

lis = rospy.init_node('LD06pose_subscriber_1', anonymous=True)
# rospy.Subscriber("/tf",TFMessage,newOdom1)
rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
pub = rospy.Publisher('chatter_LD360',String, queue_size=10)  # 发布消息到话题 chatter 中,队列长度10
rate = rospy.Rate(10)
# 循环等待回调函数
rospy.spin()


