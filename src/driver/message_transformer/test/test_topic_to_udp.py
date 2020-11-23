#!/bin/python
# coding: utf-8
# author: jsnjhhy@126.com

import socket
import struct
import rospy
import select
import time


class UdpServer:

    def __init__(self,local_port=43893):
        self.local_port = local_port
        self.server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM,0)
        self.server.bind(('0.0.0.0',self.local_port))


    def receiveMsg(self,time_out = 100):
        while not rospy.is_shutdown():
            # Receive msg with timeout.
            is_ready =  select.select([self.server],[],[],time_out)
            if is_ready[0]:
                #TODO: Currently, it's an unsafe implentment to get msgs.
                data,addr = self.server.recvfrom(2048)
                if not data :#or bytes(data).__len__()<4*3:
                    print "package broken."
                    print("package size: %d"%bytes(data).__len__())
                    continue
                else:
                    print("package size: %d"%bytes(data).__len__())
                    print("received: ",data,"from: ",addr)

                    msg_code,msg_val,msg_type = None,None,None
                    if bytes(data).__len__() == 36: 
                        (msg_code,msg_val,msg_type,a,b,c) = struct.unpack("<3i3d",data)
                        print (msg_code,msg_val,msg_type,a,b,c) 
                    else:
                        (msg_code,msg_val,msg_type,a,b,c,d) = struct.unpack("<3i4d",data)
                        print (msg_code,msg_val,msg_type,a,b,c,d)
                    return (msg_code,msg_val,msg_type)
            else:
                print "timeout!"
                return None,None,None


if __name__ == '__main__':
    rospy.init_node("test_topic_to_udp")
    udp_server = UdpServer()
    while not rospy.is_shutdown():
        udp_server.receiveMsg()
    