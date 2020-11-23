#!/usr/bin/env python
# coding: utf-8
# author: jsnjhhy@126.com

import sys
import rospy
from PyQt5.QtWidgets import (
    QWidget,
    QApplication,
    QLabel,
    QHBoxLayout,
    QVBoxLayout,
    QPushButton,
    QLineEdit,
    QTextEdit,
    QCheckBox,
)
from geometry_msgs.msg import PoseStamped
from pathlib import Path

import json
import os
import tf


class LocationRecorder(QWidget):
    def __init__(self):
        super(LocationRecorder, self).__init__()
        self.init_ui()
        self.poseSub = rospy.Subscriber("/ndt/current_pose", PoseStamped, self.poseCB)
        self.current_robot_pose = None
        self.robot_record_pose = None
        self.camera_record_pose = None
        self.option = None
        self.tf_listener = tf.TransformListener()
        self.show()

    def init_ui(self):
        self.layout = QVBoxLayout()
        self.option_layout = QHBoxLayout()
        self.up_stair_checkbox = QCheckBox("上台阶")
        self.down_stair_checkbox = QCheckBox("下台阶")
        self.middle_point_checkbox = QCheckBox("中间点")

        self.perform_checkbox = QCheckBox("表演")
        self.crawl_checkbox = QCheckBox("匍匐")
        self.walking_bricks_checkbox = QCheckBox("走砖块")

        self.speed_up_checkbox = QCheckBox("加速")

        self.option_layout.addWidget(self.up_stair_checkbox)
        self.option_layout.addWidget(self.down_stair_checkbox)
        self.option_layout.addWidget(self.middle_point_checkbox)
        self.option_layout.addWidget(self.speed_up_checkbox)

        self.option_layout.addWidget(self.perform_checkbox)
        self.option_layout.addWidget(self.crawl_checkbox)
        self.option_layout.addWidget(self.walking_bricks_checkbox)

        self.order_layout = QHBoxLayout()
        self.order_layout.addWidget(QLabel("位点编号:"))
        self.order_edit = QLineEdit("")
        self.order_layout.addWidget(self.order_edit)

        self.text_content = QTextEdit()
        self.text_content.setEnabled(False)

        self.record_layout = QHBoxLayout()
        self.receive_button = QPushButton("获取位点")
        self.record_button = QPushButton("记录位点")
        self.record_layout.addWidget(self.receive_button)
        self.record_layout.addWidget(self.record_button)

        self.layout.addLayout(self.option_layout)
        self.layout.addLayout(self.order_layout)
        self.layout.addWidget(self.text_content)
        self.layout.addLayout(self.record_layout)
        # self.layout.addStretch()
        self.setLayout(self.layout)

        self.record_button.clicked.connect(self.record)
        self.receive_button.clicked.connect(self.receive)

    def record(self):
        order = self.order_edit.text()
        try:
            order = int(order)
        except:
            # message box some error
            return

        new_record = {
            "order": order,
            "robot_pose": self.robot_record_pose,
            "camera_pose": self.camera_record_pose,
            "option": self.option,
        }
        data_dir = str(Path(__file__).parent.absolute()) + "/../data"
        os.system("mkdir -p " + data_dir)
        data_dir = data_dir + "/%d.json"
        with open(
            data_dir % order, "w+"
        ) as out:
            json.dump(new_record, out, indent=4)

    def listen_tf(self):
        try:
            (pos, ori) = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Duration(0.0)
            )
            msg_dict = {
                "pos_x": pos[0],
                "pos_y": pos[1],
                "pos_z": pos[2],
                "ori_x": ori[0],
                "ori_y": ori[1],
                "ori_z": ori[2],
                "ori_w": ori[3],
            }
            self.current_robot_pose = msg_dict
            return True
        except tf.Exception as e:
            print "listen to tf failed"
            return False

    def update_camera_state(self):
        """
            Call ptz_status to get camera pan-tilt-zoom info.
        """
        record_camera = False
        if record_camera:
            rospy.wait_for_service("ptz_status")
            try:
                ptz_status = rospy.ServiceProxy("ptz_status", PtzStatus)
                resp = ptz_status()
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
                self.camera_record_pose = None
            self.camera_record_pose = {
                "pan": resp.nPTZPan,
                "tilt": resp.nPTZTilt,
                "zoom": resp.nPTZZoom,
            }
        else:
            self.camera_record_pose = {"pan": 0, "tilt": 0, "zoom": 0}

    def update_option(self):
        self.option = {}
        self.option["up_stair"] = self.up_stair_checkbox.isChecked()
        self.option["down_stair"] = self.down_stair_checkbox.isChecked()
        self.option["middle_point"] = self.middle_point_checkbox.isChecked()
        self.option["speed_up"] = self.speed_up_checkbox.isChecked()

        self.option["perform"] = self.perform_checkbox.isChecked()
        self.option["crawl"] = self.crawl_checkbox.isChecked()
        self.option["walking_bricks"] = self.walking_bricks_checkbox.isChecked()

    def receive(self):
        while not self.listen_tf():
            rospy.sleep(1.0)
        self.robot_record_pose = self.current_robot_pose
        self.update_camera_state()
        self.update_option()
        display_msg = "Robot:\n" + json.dumps(self.robot_record_pose, indent=4) + "\n"
        display_msg += (
            "Camera:\n" + json.dumps(self.camera_record_pose, indent=4) + "\n"
        )
        display_msg += "Option:\n" + json.dumps(self.option, indent=4) + "\n"

        self.text_content.setText(display_msg)

    def poseCB(self, msg):
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        ori = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        msg_dict = {
            "pos_x": pos[0],
            "pos_y": pos[1],
            "pos_z": pos[2],
            "ori_x": ori[0],
            "ori_y": ori[1],
            "ori_z": ori[2],
            "ori_w": ori[3],
        }
        # self.current_robot_pose = msg_dict


if __name__ == "__main__":
    app = QApplication(sys.argv)
    rospy.init_node("location_recorder")
    lr = LocationRecorder()
    sys.exit(app.exec_())
