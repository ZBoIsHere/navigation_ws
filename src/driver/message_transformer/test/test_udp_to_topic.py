#!/bin/python
# coding: utf-8
# author: jsnjhhy@126.com

import socket
import struct
import rospy
import select
import time


class UdpClient:

    def __init__(self,local_port=30000,ctrl_ip='0.0.0.0',ctrl_port=25000):
        
        self.client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM,0)
        self.client.bind(('0.0.0.0',local_port))
        self.ctrl_addr = (ctrl_ip,ctrl_port)

    def send(self, command_code=234, command_size = 0, command_type = 1):
        data = struct.pack('<3I6d', command_code, 60-12, command_type,0.1,0.2,0.3,0.4,0.5,0.6)
        self.client.sendto(data,self.ctrl_addr)


if __name__ == '__main__':
    rospy.init_node("test_udp_to_topic")
    udp_client1 = UdpClient(local_port=30000,ctrl_port=25000)
    udp_client2 = UdpClient(local_port=30001,ctrl_port=25001)
    while not rospy.is_shutdown():
        udp_client1.send()
        rospy.sleep(0.1)
        udp_client2.send()
        rospy.sleep(0.1)
    