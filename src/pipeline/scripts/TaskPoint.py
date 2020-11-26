#!/usr/bin/python
"""
Copyright (c) Deep Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
"""
import time
from tf import transformations
from RobotCommander import RobotCommander
import rospy

# from hik_ptz_camera.srv import PtzCtrl
import tf
import copy
from RecognizeServer import recognize_server


class TaskPoint:
    """Property and method about a task point.

    Attributes:
        pose: the pose of robot in this wappoint.
        name: waypoint name.
    """
    def __init__(self, record=None):
        if not record:
            record = {
                "order": 0,
                "robot_pose": {
                    "pos_x": 0.0,
                    "pos_y": 0.0,
                    "pos_z": 0.0,
                    "ori_x": 0.0,
                    "ori_y": 0.0,
                    "ori_z": 0.0,
                    "ori_w": 1.0
                },
                "camera_pose": {"pan": 0, "tilt": 150, "zoom": 0},
                "option": {
                    "up_stair": False,
                    "down_stair": False,
                    "middle_point": False,
                    "speed_up": False,
                    "perform": False,
                    "jump1": False,
                    "jump2": False,
                    "crawl": False,
                    "openDoor": False,
                    "leaveDoor": False,
                    "autoCharge": False,
                    "climbSlope": False,
                    "walking_bricks": False
                },
            }
        self.pre_task_point = None
        self.record = copy.deepcopy(record)
        self.tf_listener = tf.TransformListener()
        self.update()

    def setPreTaskPoint(self, src_point):
        self.pre_task_point = src_point

    def getPreTaskPoint(self):
        return self.pre_task_point

    def is_up_stair(self):
        return self.record["option"]["up_stair"]

    def is_down_stair(self):
        return self.record["option"]["down_stair"]

    def is_middle_point(self):
        return self.record["option"]["middle_point"]

    def is_speed_up(self):
        return self.record["option"]["speed_up"]

    def is_perform(self):
        return self.record["option"]["perform"]

    def is_jump1(self):
        if not self.record["option"].has_key("jump1"):
            return False
        return self.record["option"]["jump1"]

    def is_jump2(self):
        if not self.record["option"].has_key("jump2"):
            return False
        return self.record["option"]["jump2"]

    def is_crawl(self):
        if not self.record["option"].has_key("crawl"):
            return False
        return self.record["option"]["crawl"]

    def is_openDoor(self):
        if not self.record["option"].has_key("openDoor"):
            return False
        return self.record["option"]["openDoor"]

    def is_leaveDoor(self):
        if not self.record["option"].has_key("leaveDoor"):
            return False
        return self.record["option"]["leaveDoor"]

    def is_autoCharge(self):
        if not self.record["option"].has_key("autoCharge"):
            return False
        return self.record["option"]["autoCharge"]

    def is_climbSlope(self):
        if not self.record["option"].has_key("climbSlope"):
            return False
        return self.record["option"]["climbSlope"]

    def is_walking_bricks(self):
        return self.record["option"]["walking_bricks"]

    def update(self):
        self.robot_pose = self.record["robot_pose"]
        self.camera_pose = self.record["camera_pose"]
        pose = []
        pose.append(self.robot_pose["pos_x"])
        pose.append(self.robot_pose["pos_y"])
        pose.append(self.robot_pose["pos_z"])
        pose.append(self.robot_pose["ori_x"])
        pose.append(self.robot_pose["ori_y"])
        pose.append(self.robot_pose["ori_z"])
        pose.append(self.robot_pose["ori_w"])
        self.posX = pose[0]
        self.posY = pose[1]
        self.yaw = transformations.euler_from_quaternion(pose[3:])[2]
        self.name = "waypoint_" + str(self.record["order"])

    def setRobotPose(self, robot_pose):
        self.record["robot_pose"]["pos_x"] = robot_pose[0]
        self.record["robot_pose"]["pos_y"] = robot_pose[1]
        self.record["robot_pose"]["pos_z"] = robot_pose[2]

        self.record["robot_pose"]["ori_x"] = robot_pose[3]
        self.record["robot_pose"]["ori_y"] = robot_pose[4]
        self.record["robot_pose"]["ori_z"] = robot_pose[5]
        self.record["robot_pose"]["ori_w"] = robot_pose[6]
        self.update()

    def getPosX(self):
        return self.posX

    def getPosY(self):
        return self.posY

    def getYaw(self):
        return self.yaw

    def calDistance(self, other):
        return (
            (self.getPosX() - other.getPosX()) ** 2
            + (self.getPosY() - other.getPosY()) ** 2
        ) ** 0.5

    def adjustCameraExactPose(self, pan=None, tilt=None, zoom=None):
        print ("[{}]Adjust camera to predefine Camera Pose.".format(self.name))
        rospy.wait_for_service("ptz_ctrl")
        try:
            ptz_ctrl = rospy.ServiceProxy("ptz_ctrl", PtzCtrl)
            if None in [pan, tilt, zoom]:
                pan = self.camera_pose["pan"]
                tilt = self.camera_pose["tilt"]
                zoom = self.camera_pose["zoom"]
            resp = ptz_ctrl(2, pan, 0, zoom)
            rospy.sleep(0.5)
            resp = ptz_ctrl(2, pan, tilt, zoom)
            print resp.status_message
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            self.camera_record_pose = None
        rospy.sleep(3.0)

    def cameraPoseReset(self):
        reset_camera_pose = {"pan": 0, "tilt": 150, "zoom": 0}
        self.adjustCameraExactPose(
            pan=reset_camera_pose["pan"],
            tilt=reset_camera_pose["tilt"],
            zoom=reset_camera_pose["zoom"],
        )

    def adjustRobotPose(self):
        rospy.sleep(1.0)
        robot_current_pose = None
        RATE = 20
        while not robot_current_pose:
            robot_current_pose = self.listen_tf()
            rospy.sleep(1.0 / RATE)
        current_yaw = transformations.euler_from_quaternion(robot_current_pose[3:])[2]
        yaw_diff = self.getYaw() - current_yaw
        print "----------------------------------------"
        print "Yaw diff: ", yaw_diff
        print "PosX diff: ", self.getPosX() - robot_current_pose[0]
        print "PosY diff: ", self.getPosY() - robot_current_pose[1]
        print "----------------------------------------"
        with RobotCommander() as robot_commander:
            robot_commander.yaw_adjust(yaw_diff)
            rospy.sleep(4.0)

    def adjustCameraRelativePose(self, object_id_list):
        print (
            "[{}]Adjust camera according to the small-feekback-loop.(Currently do nothing)".format(
                self.name
            )
        )
        recognize_server.object_fastgo(object_id_list[0])
        rospy.sleep(2.0)

    def waitForTaskFinish(self, time_out=30):
        print (
            "[{}]Wait for task finish command,the big-feedback-loop.(Currently do nothing)".format(
                self.name
            )
        )
        rospy.sleep(3.0)

    def listen_tf(self):
        try:
            (pos, ori) = self.tf_listener.lookupTransform(
                "/map", "/base_link", rospy.Duration(0.0)
            )
            print "pos: ", pos
            print "ori: ", ori
            msg_list = [pos[0], pos[1], pos[2], ori[0], ori[1], ori[2], ori[3]]
            return msg_list
        except tf.Exception as e:
            print "listen to tf failed"
            print e
            print e.message
            return None

    def runTask(self):
        if (
            self.is_middle_point()
        ):  # and not (self.is_down_stair() or self.is_up_stair()):
            return
        with RobotCommander() as robot_commander:
            robot_commander.motion_start_stop()
            rospy.sleep(4.0)
        # robot_commander.dance()
        # print("[{}]shoot image and send out.".format(self.name))
        # rospy.sleep(5.0)
        # robot_commander.dance()
        with RobotCommander() as robot_commander:
            robot_commander.motion_start_stop()
            rospy.sleep(2.0)


def globalTaskPrepare():
    with RobotCommander() as robot_commander:
        robot_commander.stand_down_up()
        rospy.sleep(5.0)
        robot_commander.start_force_mode()
        rospy.sleep(2.0)
        robot_commander.motion_start_stop()
        rospy.sleep(2.0)


def globalTaskFinish():
    with RobotCommander() as robot_commander:
        robot_commander.motion_start_stop()
        rospy.sleep(2.0)
        robot_commander.stand_down_up()
        rospy.sleep(5.0)
