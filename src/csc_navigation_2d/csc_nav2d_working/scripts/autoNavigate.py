#!/usr/bin/env python
# -- coding: utf-8 --

import time
import socket
import struct
import rospy
import roslib
import actionlib
import tf
import dynamic_reconfigure.client
from tf import transformations

from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from csc_nav2d_navigator.msg import MoveToPosition2DAction, MoveToPosition2DGoal

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 43894))  # 本地端口
targetAddr = ('192.168.1.120', 43893)  # 目标IP和端口

linear_vel = 0.0  # 目标线速度
angular_vel = 0.0  # 目标角速度
obstacles_range = 0.0  # 正前方障碍物的距离
in_region = False  # 是否处于特殊区域中的标志
odom_data = Odometry()

MoveTo_client = actionlib.SimpleActionClient('MoveTo', MoveToPosition2DAction)

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
    #linear_y = -odom_data.twist.twist.linear.y
    linear_y = 0.0
    angular_vel = msg.angular.z

    if MoveTo_client.get_state() == 3 :
        goal_state = 1
    else :
        goal_state = 0

    str1 = '%.3f' % linear_x
    str2 = '%.3f' % linear_y
    str3 = '%.3f' % angular_vel
    str4 = '%.3f' % obstacles_range
    str5 = '%d' % in_region
    str6 = '%d' % goal_state
    msg = str1 + ',' + str2 + ',' + str3 + ',' + str4 + ',' + str5 + ',' + str6
    code = 14
    code_bytes = struct.pack('<I',code)
    len_bytes = struct.pack('<I',len(msg))

    s.sendto(code_bytes+len_bytes+'\x01\x00\x00\x00'+msg, targetAddr)
    #print msg

def listener():

    rospy.init_node('autoNavigation', anonymous=True)

    rospy.Subscriber('cmd_vel', Twist, vel_callback)
    rospy.Subscriber('obstacles_range', Float32, obstacles_callback)
    rospy.Subscriber('in_region', Bool, inregion_callback)
    rospy.Subscriber('odom', Odometry, odom_callback)
    
    initPose_x = rospy.get_param("~initPose_x", 0.0)
    initPose_y = rospy.get_param("~initPose_y", 0.0)
    initPose_th = rospy.get_param("~initPose_th", 0.0)
    goalPose_x = rospy.get_param("~goalPose_x", 0.0)
    goalPose_y = rospy.get_param("~goalPose_y", 0.0)
    goalPose_th = rospy.get_param("~goalPose_th", 0.0)

    initpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    time.sleep(5)

    # 设定初始位置
    q = tf.transformations.quaternion_from_euler(0, 0, initPose_th)
    initpose_msg = PoseWithCovarianceStamped()
    initpose_msg.header.stamp = rospy.Time.now()
    initpose_msg.header.frame_id = "map"
    initpose_msg.pose.pose.position.x = initPose_x
    initpose_msg.pose.pose.position.y = initPose_y
    initpose_msg.pose.pose.position.z = 0
    initpose_msg.pose.pose.orientation.x = q[0]
    initpose_msg.pose.pose.orientation.y = q[1]
    initpose_msg.pose.pose.orientation.z = q[2]
    initpose_msg.pose.pose.orientation.w = q[3]
    initpose_pub.publish(initpose_msg)  #发布初始位置
    print 'init_pose: '+str(initPose_x)+'  '+str(initPose_y)+'  '+str(initPose_th)
    
    MoveTo_client.wait_for_server()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #receviedData, addrInfo = s.recvfrom(2048)
        # 设定第一个目标位置
        goal_msg = MoveToPosition2DGoal()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.target_pose.x = goalPose_x
        goal_msg.target_pose.y = goalPose_y
        goal_msg.target_pose.theta = goalPose_th
        MoveTo_client.send_goal(goal_msg)
        print 'goal_pose: ' + str(goal_msg.target_pose.x) + '  ' + str(goal_msg.target_pose.y) + '  ' + str(goal_msg.target_pose.theta)

        MoveTo_client.wait_for_result()
        print 'Arrived the first goal.' + str(MoveTo_client.get_state())

        # 到达第一个目标位置后回到起点
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.target_pose.x = initPose_x
        goal_msg.target_pose.y = initPose_y
        goal_msg.target_pose.theta = initPose_th
        MoveTo_client.send_goal(goal_msg)
        print 'goal_pose: ' + str(goal_msg.target_pose.x) + '  ' + str(goal_msg.target_pose.y) + '  ' + str(goal_msg.target_pose.theta)

        MoveTo_client.wait_for_result()
        print 'Arrived the second goal.'
        time.sleep(5)

    
if __name__ == '__main__':
    try:
        listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
