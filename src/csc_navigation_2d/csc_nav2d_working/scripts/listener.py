#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import socket
import struct

from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 43894))  # 本地端口
targetAddr = ('192.168.1.120', 43893)  # 目标IP和端口

linear_vel = 0.0  # 目标线速度
angular_vel = 0.0  # 目标角速度
obstacles_range = 0.0  # 正前方障碍物的距离
in_region = False  # 是否处于特殊区域中的标志
odom_data = Odometry()

def obstacles_callback(msg):
    global obstacles_range
    obstacles_range = msg.data

def inregion_callback(msg):
    global in_region
    in_region = msg.data

def odom_callback(msg):
    global odom_data
    odom_data = msg

def vel_callback(msg):
    linear_x = msg.linear.x
    linear_y = -odom_data.twist.twist.linear.y
    angular_vel = msg.angular.z

    str1 = '%.3f' % linear_x
    str2 = '%.3f' % linear_y
    str3 = '%.3f' % angular_vel
    str4 = '%.3f' % obstacles_range
    str5 = '%d' % in_region
    msg = str1 + ',' + str2 + ',' + str3 + ',' + str4 + ',' + str5
    code = 14
    code_bytes = struct.pack('<I',code)
    len_bytes = struct.pack('<I',len(msg))

    s.sendto(code_bytes+len_bytes+'\x01\x00\x00\x00'+msg, targetAddr)
    #print msg

def listener():

    rospy.init_node('udpSender', anonymous=True)
    
    rospy.Subscriber('cmd_vel', Twist, vel_callback)
    rospy.Subscriber('obstacles_range', Float32, obstacles_callback)
    rospy.Subscriber('in_region', Bool, inregion_callback)
    rospy.Subscriber('odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #receviedData, addrInfo = s.recvfrom(2048)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
