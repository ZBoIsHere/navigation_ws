#!/usr/bin/python
# coding: utf-8
# author: jsnjhhy@126.com

import rospy
import json
import threading
import socket
import struct
import array
from ctypes import create_string_buffer
from collections import OrderedDict
import copy
import select
import os



class Topic2UDPConverter:
    def __init__(self,config,local_port,server_ip='192.168.1.120',server_port=43893):
        topic_name = config["topic_name"]
        msg_type = config["msg_type"]
        struct_mapping = config["mapping"]
        command_code = config["command_code"]

        self.msg_type = msg_type
        self.struct_mapping = struct_mapping
        self.command_code = command_code
        self.client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM,0)
        self.client.bind(('0.0.0.0',local_port))
        self.server_addr = (server_ip,server_port)
        exec("self.topic_sub = rospy.Subscriber('"+topic_name+"',"+msg_type+","+"self.msg_cb, queue_size=5)")
        

    def msg_cb(self,msg):
        _offset = 0
        data = array.array('c')
        data.extend(" "*12)
        struct.pack_into('<3i',data,_offset,self.command_code,0,1)
        _offset = data.__len__()

        for key in self.struct_mapping:
            value = self.struct_mapping[key]
            # 考虑4字节对齐
            len = 4
            if value in ['c','b','B','?','s']:
                len = 4 #1
            elif value in ['h','H']:
                len = 4 #2
            elif value in ['i','I','l','L','f']:
                len = 4 #4
            elif value in ['q','Q','d']:
                len = 8
            else:
                raise Exception("Invalid struct format!")
                exit(1)
            data.extend(" "*len)
            print("struct.pack_into('<"+value+"',data,_offset,msg"+key+")")
            exec("struct.pack_into('<"+value+"',data,_offset,msg"+key+")")
            _offset += len
        # put packet_size.
        struct.pack_into('<i',data,4,data.__len__()-12)
        self.client.sendto(data,self.server_addr)

class UDP2TopicConverter:
    def __init__(self,local_port):
        self.local_port = local_port
        self.udp_listener = socket.socket(socket.AF_INET,socket.SOCK_DGRAM,0)
        self.udp_listener.bind(('0.0.0.0',local_port))
        self.code_mapping = {}
        self.listener_thread = threading.Thread(target=self.receiveMsg)
        self.listener_thread.setDaemon(True)
        self.listener_thread_started = False

    def get_topic_clear_name(self,topic_name):
        return topic_name.replace('/','')

    def add_topic(self,config):
        if self.code_mapping.has_key(config["command_code"]):
            self.code_mapping[config["command_code"]][config["topic_name"]]=copy.deepcopy(config)
        else:
            self.code_mapping[config["command_code"]] = {  config["topic_name"]:copy.deepcopy(config)}
        print(self.code_mapping)
        exec("self."+self.get_topic_clear_name(config["topic_name"])+"_pub = rospy.Publisher('"+config["topic_name"]+"',"+config["msg_type"]+",queue_size=5)")
        
    def run(self):
	# TODO: better  thread start logic.
        if not self.listener_thread_started:
            self.listener_thread.start()
            self.listener_thread_started = True
        

    def publish_msg(self,command_code,data):
        _offset = 12
        # check if commad_code is configured.
        if not self.code_mapping.has_key(command_code):
            raise Exception("Receive command_code %d from UDP which is undefined in config file!"%(command_code))
        # publish every topic from this package.
        for topic_name in self.code_mapping[command_code]:
            topic_config = self.code_mapping[command_code][topic_name]
	    print("msg = " + topic_config["msg_type"] + "()")
            exec("msg = " + topic_config["msg_type"] + "()")
	    if topic_config.has_key("additional_cmd"):
		print(topic_config["additional_cmd"])
		exec(topic_config["additional_cmd"])
            offset_mapping = topic_config["offset_mapping"]
            type_mapping = topic_config["type_mapping"]
            for key in offset_mapping:
                offset,value = offset_mapping[key],type_mapping[key]
		print("msg"+key+"=struct.unpack_from('<"+value+"',data,_offset+offset)[0]")
                exec("msg"+key+"=struct.unpack_from('<"+value+"',data,_offset+offset)[0]")
                # exec("print msg"+key)
            print("self."+self.get_topic_clear_name(topic_config["topic_name"])+"_pub.publish(msg)")
            exec("self."+self.get_topic_clear_name(topic_config["topic_name"])+"_pub.publish(msg)")
            


    def receiveMsg(self,time_out = 5):
        while not rospy.is_shutdown():
            # Receive msg with timeout.
            is_ready =  select.select([self.udp_listener],[],[],time_out)
            if is_ready[0]:
                #TODO: Currently, it's an unsafe implentment to get msgs.
                data,addr = self.udp_listener.recvfrom(2048)
                if not data:
                    print "package broken."
                    print("package size: %d"%bytes(data).__len__())
                    continue
                else:
                    print("package size: %d"%bytes(data).__len__())
                    (msg_code,msg_size,msg_type) = struct.unpack_from("<3I",data)
                    print "msg_type",msg_type
                    print "msg_size",msg_size
                    if msg_type!=1 or msg_size+12!=bytes(data).__len__() or msg_size == 40:
                        continue
                    print("received: ",data,"from: ",addr)
                    self.publish_msg(msg_code,data)
            else:
                print("timeout!")

if __name__ == '__main__':
    rospy.init_node("topic_to_udp")
    cfg_path =os.path.dirname(os.path.abspath(__file__)) + "/../config/rule.json"
    with open(cfg_path,'r') as input:
        config_list = json.load(input,object_pairs_hook=OrderedDict)

    # print "Size of config_list: ", len(config_list)
    topic_coverter_list = []
    udp_coverter_list = []
    local_send_port = 20000
    for config in config_list:
        exec("from " + config["msg_pkg"]+" import "+config["msg_type"])
        if config["convert"]["src"] == "topic" and config["convert"]["des"] == "udp":
            topic_coverter_list.append(Topic2UDPConverter(config,local_port=local_send_port))
            local_send_port +=1
        if config["convert"]["src"] == "udp" and config["convert"]["des"] == "topic":
            port_used = False
            for converter in udp_coverter_list:
                if converter.local_port == config["local_port"] :
                    converter.add_topic(config)
                    port_used = True
            if not port_used:
                udp_coverter_list.append(UDP2TopicConverter(local_port=config["local_port"]))
                udp_coverter_list[-1].add_topic(config)
    # print "Size of topic_coverter_list: ", len(topic_coverter_list)
    # print "Size of udp_coverter_list: ", len(udp_coverter_list)
    for converter in udp_coverter_list:
        converter.run()

    rospy.spin()
