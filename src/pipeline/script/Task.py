#!/usr/bin/python
"""
Copyright (c) Deep Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
"""

import os
import re
import json
import rospy
from TaskPoint import TaskPoint, globalTaskPrepare
from TaskTransfer import TaskTransfer
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf
from RobotCommander import RobotCommander
import threading


class Task:
    def __init__(self):
        self.taskPoints = []
        self.currentIndex = 0
        self.robot_transfer = None
        self.src_ind = None
        self.des_ind = None
        self.ntask = 0
        self.tf_listener = tf.TransformListener()
        self.send_tf_thread = threading.Thread(target=self.send_tf, name="send_tf")
        self.send_tf_thread.setDaemon(True)
        self.send_tf_thread.start()

    def send_tf(self):
        while not rospy.is_shutdown():
            current_tf = self.listen_tf()
            if current_tf:
                # print("*********************************")
                # print("send_tf")
                with RobotCommander(local_port=20003) as robot_commander:
                    robot_commander.sendCordinate(
                        command_code=52,
                        x=current_tf[0],
                        y=current_tf[1],
                        yaw=current_tf[2],
                    )
            rospy.sleep(0.05)

    def listen_tf(self):
        try:
            (pos, ori) = self.tf_listener.lookupTransform(
                "/map", "/base_link", rospy.Duration(0.0)
            )
            # print "pos: ",pos
            # print "ori: ",ori
            yaw = tf.transformations.euler_from_quaternion(ori)[2]
            msg_list = [pos[0], pos[1], yaw]
            return msg_list
        except tf.Exception as e:
            print "listen to tf failed"
            print e
            print e.message
            return None

    def init(self):
        globalTaskPrepare()
        task_init = TaskInit()
        self.robot_transfer = TaskTransfer()
        self.loadTaskpoints()
        best_ind, initial_point = task_init.getBestTaskInd(self.taskPoints)
        # import pdb;pdb.set_trace()
        # First to transfer to best index point 'self.taskPoints[best_index]'
        self.robot_transfer.task_transfer(initial_point, self.taskPoints[best_ind])
        self.ntask = self.taskPoints.__len__()
        self.src_ind = best_ind
        self.des_ind = (best_ind + 1) % self.ntask

    def task_cmp(self, t1, t2):
        return t1["order"] < t2["order"]

    def loadTaskpoints(self):
        folder = str(os.path.dirname(os.path.abspath(__file__))) + "/../data"
        task_json = None
        if os.path.exists(folder):
            task_json = os.listdir(folder)

        if not task_json:
            raise Exception("No valid task point to tranverse!!")

        task_list = []
        for i, file_name in enumerate(task_json):
            with open(folder + "/" + file_name, "r") as json_fp:
                waypoint_record = json.load(json_fp)
                task_list.append(waypoint_record)
        task_list = sorted(task_list, key=lambda s: s["order"])
        for waypoint_record in task_list:
            self.taskPoints.append(TaskPoint(waypoint_record))

    def filterUnrelatedData(self, task_json):
        filtered = []
        for file_name in task_json:
            if re.sub("[\d,-]", "", file_name) == ".json":
                filtered.append(file_name)
        task_json = filtered

    def run(self):
        while not rospy.is_shutdown():
            self.robot_transfer.task_transfer(
                self.taskPoints[self.src_ind], self.taskPoints[self.des_ind]
            )
            # self.taskPoints[self.des_ind].runTask()
            self.src_ind = self.des_ind
            self.des_ind = (self.des_ind + 1) % self.ntask


class TaskInit:
    def __init__(self, pose_topic="/ndt/current_pose"):
        self.initialPose = None
        self.pose_topic = pose_topic
        self.tf_listener = tf.TransformListener()

    def poseCallback(self, msg):
        self.initialPose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

    def listen_tf(self):
        try:
            (pos, ori) = self.tf_listener.lookupTransform(
                "/map", "/base_link", rospy.Duration(0.0)
            )
            print "pos: ", pos
            print "ori: ", ori
            msg_list = [pos[0], pos[1], pos[2], ori[0], ori[1], ori[2], ori[3]]
            self.initialPose = msg_list
            return True
        except tf.Exception as e:
            print "listen to tf failed"
            print e
            print e.message
            return False

    def refreshInitialPose(self):
        self.initialPose = None
        # rospy.Subscriber(self.pose_topic,PoseStamped,self.poseCallback)
        RATE = 50
        while not self.initialPose:
            self.listen_tf()
            rospy.sleep(1.0 / RATE)

    def getBestTaskInd(self, task_points):
        self.refreshInitialPose()
        fake_task = TaskPoint()
        fake_task.setRobotPose(self.initialPose)
        dist_list = [fake_task.calDistance(task_point) for task_point in task_points]
        # import pdb;pdb.set_trace()
        return np.argmin(np.array(dist_list)), fake_task


if __name__ == "__main__":
    rospy.init_node("autonomous_2d_navigation")
    task = Task()
    task.init()
    task.run()
