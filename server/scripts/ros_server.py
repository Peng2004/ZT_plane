#!/usr/bin/python
#-*- coding:utf-8 -*-

import os # 操作系统
import sys # 与python解释器交互的一个接口
import tty # 提供终端相关的接口
import termios # 终端控制
import socket # 套接字
import threading # 线程
import multiprocessing # 多线程
# import Queue # 队列，用于多线程通信
#from queue import Queue
import queue

import roslaunch
import time
from time import sleep
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose # 姿势，包含位置和方向
from geometry_msgs.msg import PoseArray # 姿势数组
from geometry_msgs.msg import PoseStamped # 一个带有参考坐标系和时间戳的Pose
from geometry_msgs.msg import PoseWithCovarianceStamped # 这表示带有参考坐标系和时间戳的估计位姿
from nav_msgs.msg import Odometry # 这表示对自由空间中位置和速度的估计
from nav_msgs.msg import Path #代表机器人跟随路径的姿势(PoseStamped)数组
from nav_msgs.srv import GetPlan # 得到一个从当前位置到目标的计划 Pose
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseGoal
from detection_msgs.msg import position
# from qingzhou_description.msg import position
from std_srvs.srv import Empty
from std_msgs.msg import Int16 # short
from std_msgs.msg import UInt8 # unsigned char
import numpy as np # 使用as语法简化引用numpy

#服务器线程
"""
    该线程的任务是
    1、解码接收无人机传回的图像信息

"""
class ScokServer(threading.Thread):

    def _init_(self,host_ip,host_port,pipe):
       # 线程初始化
        threading.Thread.__init__(self)
        # 运行标志位初始化
        self.running=False
        # 定义服务器的IP地址和端口号
    server_ip = '127.0.0.1'  # 可以监听所有网络接口
    server_port = 12345  # 选择一个未被占用的端口号

    # 创建TCP套接字对象
    self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定IP地址和端口号
    self.server_sock.bind((server_ip, server_port))

    # 开始监听连接
    self.server_sock.listen(5)  # 最大连接数为5

    print("Server is listening on %s:%d" % (server_ip, server_port))

    while not self.running:
        # 等待客户端连接
        self.client_sock, self.client_addr = self.server_sock.accept()
        print("Connection from", self.client_addr)

        try:
            # 接收数据
            self.data = self.client_sock.recv(1024)  # 一次最多接收1024字节的数据
            if self.data:
                #print("Received:", data.decode('utf-8'))  # 将接收到的数据解码为UTF-8格式并打印
                # 这里可以添加处理收到数据的逻辑

                # 发送响应数据
                client_sock.sendall(b"Message received!")  # 发送一条简单的响应消息
        except Exception as e:
            print("Error:", e)
        finally:
            # 关闭客户端连接
            self.client_sock.close()
    # 线程的主函数，当线程调用start()方法时，会自动运行run函数
    def run(self):
        # 如果客户端成功连接，循环会一直运行
        while self.running:
            try:
                data = self.lient_sock.recv(1024) # 接收套接字 (Tcp服务器) 的数据
                # 调用类成员函数decode_rx对数据报进行解码
                cmd=self.decode_rx(data)
                print('socet_cmd:\n',cmd)
                # 通过管道发送cmd
                self.pipe.send(cmd)
                print('send to pip\n')
                # 如果管道里面没有数据
                if not self.pipe.poll():
                    self.sock.send(b'no msg yet')
                    rospy.loginfo('NO MSG')
                    sleep(0.05)
                    continue
                # 收取管道的信息
                pipe_rx = self.pipe.recv()

            # 如果连接断开，循环退出       
            except Exception as e:
                print ('socket running error:\n', str(e)) # 加了括号
                os._exit(0)
                break

        rospy.loginfo('SockServer Thread Exit')

    # 对服务端发送的数据报进行解码接收
    def decode_rx(self,rx_data):
        # 区分不同指令的标签
        ret=('cmd','pause')
        param = []

        #切片方式匹配
        expected_header = b'\x02\x20\x02\x20'
        index = rx_data.find(expected_header, 0, len(rx_data))
        if index == -1:
            rospy.loginfo('pipeishibai')
            return []

        else:
            rospy.loginfo('pipeichenggong')

        # 匹配成功则继续收取数据包
        frame_size = np.frombuffer(rx_data[(index+4):(index+8)], dtype=np.int32)
        # 提取数据内容
        rx_data=rx_data[(index+8):(index+8+frame_size[0])]

# ros端线程
"""
    该线程的任务是
    1.作为管道的另一端，接收来自客户端线程解码好的数据
    2.作为话题通信的发布者，向各ros节点发布整理好的指令内容
"""
class CarDispatcherROS(threading.Thread):
    def __init__(self,pipe):
        threading.Thread.__init__(self)
        # 初始化ros节点
        rospy.init_node('ros_server',anonymous=True)