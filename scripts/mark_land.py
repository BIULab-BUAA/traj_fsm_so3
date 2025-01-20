#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: xindong324
Date: 2022-09-19 19:50:43
LastEditors: xindong324 xindong324@163.com
LastEditTime: 2024-01-18 05:49:48
Description: file content
'''
import numpy as np
from numpy import ma, mat
import rospy
import tf
import math
import copy
from datetime import datetime
import time
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *
from quadrotor_msgs.msg import PositionCommand


PI = 3.1415926
YAW_DOT_MAX_PER_SEC = PI
last_yaw_ = 0
last_yaw_dot_ = 0
time_forward_ = 1.0
start_time = time.time()

traj_pub = None
key_pub = None

rcv_odom = False

mark_odom = Odometry()

traj = PositionCommand()
key_data = String()

uav_odom = Odometry()

stage = 1
# 1: flying 2 mark
# 2: decend
# 3: land



def reachedTargetPosition( tar_pos, cur_pos, thres):
    tar = np.array([tar_pos.x, tar_pos.y, tar_pos.z], dtype=np.float32)
    cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z], dtype=np.float32)
    return (np.linalg.norm(tar - cur) < thres)

def reachedTargetPosition2d( tar_pos, cur_pos, thres):
    tar = np.array([tar_pos.x, tar_pos.y], dtype=np.float32)
    cur = np.array([cur_pos.x, cur_pos.y], dtype=np.float32)
    return (np.linalg.norm(tar - cur) < thres)

def uavodomCallback(msg):
    global uav_odom
    uav_odom.pose =  msg.pose
    # print("x",mark_odom.pose.pose.position.x)
    

def markodomCallback(msg):
    global mark_odom, key_data
    mark_odom.pose =  msg.pose
    # print("x",mark_odom.pose.pose.position.x)
    
def fsm_cb(event):
    global traj, start_time,mark_odom,stage
    now = time.time()
    t = (now - start_time)
    print("t ",t)
    # if(t < 1e-6 or mark_odom == None):
    #     return
    forward_t = 1.8
    if(reachedTargetPosition2d(mark_odom.pose.pose.position, uav_odom.pose.pose.position,0.8)):
        traj.position.x = mark_odom.pose.pose.position.x  + mark_odom.twist.twist.linear.x * forward_t
        traj.position.y = mark_odom.pose.pose.position.y + mark_odom.twist.twist.linear.y * forward_t
        
        traj.position.z = uav_odom.pose.pose.position.z - 0.3
        traj.velocity.z = -0.3

        traj.velocity.x = mark_odom.twist.twist.linear.x
        traj.velocity.y = mark_odom.twist.twist.linear.y
        if(uav_odom.pose.pose.position.z < 0.1 and reachedTargetPosition2d(mark_odom.pose.pose.position, uav_odom.pose.pose.position,0.1)):
            stage = 3
    else:
        
        traj.position.x = mark_odom.pose.pose.position.x + mark_odom.twist.twist.linear.x * forward_t
        traj.position.y = mark_odom.pose.pose.position.y + mark_odom.twist.twist.linear.y * forward_t
        
        traj.position.z = 2

        # traj.velocity.x = mark_odom.twist.twist.linear.x
        # traj.velocity.y = mark_odom.twist.twist.linear.y

    traj_pub.publish(traj)
    # if(stage < 3):
    #     traj_pub.publish(traj)
    # if(stage == 3):
    #     key_data.data = "g"
        # key_pub.publish(key_data)

if __name__ == '__main__':
    # global key_pub, traj_pub, start_time
    traj_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=1)
    # traj_pub = rospy.Publisher('/pos_cmd', PositionCommand, queue_size=1)
    key_pub = rospy.Publisher('/keys', String, queue_size=1)

    rospy.init_node("markland")
    rate = rospy.Rate(100)

    mark_odom_sub = rospy.Subscriber("/ground_truth/landing_pad_aruco", Odometry, callback=markodomCallback, queue_size=1)
    uav_odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, callback=uavodomCallback, queue_size=1)

    print("Publishing keystrokes. Press Ctrl-C to exit...")
    time_last_ = time.time()
    start_time = time.time()
    last_t = 0

    exec_timer_ = rospy.Timer(rospy.Duration(0.01), fsm_cb)
    rospy.spin()
   