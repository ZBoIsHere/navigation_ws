#!/usr/bin/python3
"""
Copyright (c) Deep Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
"""


import rospy
import json
import time
import datetime
import os
from geometry_msgs.msg import PoseStamped


class PoseRecorder:
    """
        PoseRecoder subscribe to the "/current_robot_pose"('/ndt/current_pose') topic and save it to an json file with timestamp. 
    """

    def __init__(self, pose_topic: str = "/ndt/current_pose"):
        rospy.Subscriber(pose_topic, PoseStamped, self.pose_listenning)
        self.currentPose = None
        self.poseRecord = {}

    def pose_listenning(self, msg: PoseStamped):
        self.currentPose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        print(self.currentPose)

    def saveRecord(self):
        self.currentPose = None
        while not rospy.is_shutdown():
            if not self.currentPose:
                rospy.sleep(0.1)
                continue
            else:
                self.poseRecord["pose"] = self.currentPose
                self.poseRecord["task_list"] = []
                self.poseRecord["timestamp"] = int(time.time())
                record_name = (
                    datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".json"
                )
                record_file = "../data/raw/" + record_name
                os.system("touch {}".format(record_file))
                with open(record_file, "w") as output:
                    json.dump(self.poseRecord, output, indent=4)
                break


if __name__ == "__main__":
    rospy.init_node("pose_recorder")
    pose_recorder = PoseRecorder()
    pose_recorder.saveRecord()
