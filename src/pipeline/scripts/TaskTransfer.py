#!/usr/bin/python
"""
Copyright (c) Deep Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
"""

import rospy
import actionlib
from tf.transformations import *
from pipeline.msg import MoveBaseAction, MoveBaseGoal
from TaskPoint import TaskPoint
from RobotCommander import RobotCommander


class TaskTransfer:
    def __init__(self):
        self.moveBaseClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.moveBaseClient.wait_for_server()
        rospy.loginfo("Action 'move_base' is up!")

    def plan_failed(self):
        return self.moveBaseClient.get_state() == actionlib.GoalStatus.ABORTED

    def is_action_succeed(self):
        return self.moveBaseClient.get_state() == actionlib.GoalStatus.SUCCEEDED

    def task_transfer(self, src_point, des_point):
        """
        Main Decision Function
        """
        with RobotCommander() as robot_commander:
            robot_commander.sendCordinate(
                command_code=51,
                x=src_point.getPosX(),
                y=src_point.getPosY(),
                yaw=src_point.getYaw(),
            )
            print ("-----------task_transfer-----------")
        des_point.setPreTaskPoint(src_point)

        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.pose.position.x = des_point.getPosX()
        goal_msg.target_pose.pose.position.y = des_point.getPosY()
        goal_msg.target_pose.pose.position.z = 0
        my_q = quaternion_from_euler(0, 0, des_point.getYaw())
        goal_msg.target_pose.pose.orientation.x = my_q[0]
        goal_msg.target_pose.pose.orientation.y = my_q[1]
        goal_msg.target_pose.pose.orientation.z = my_q[2]
        goal_msg.target_pose.pose.orientation.w = my_q[3]

        print des_point.record["option"]

        not_done = True

        while not_done and not rospy.is_shutdown():
            """
            Do the task from src to des until SUCCEEDED/ABORTED, and trigger the special action
            """
            # TODO Need send_goal repeatedly?
            self.moveBaseClient.send_goal(goal_msg)
            rospy.logwarn(
                "Transfer from [%s] to [%s]" % (src_point.name, des_point.name)
            )

            # crawl
            if des_point.getPreTaskPoint() and des_point.getPreTaskPoint().is_crawl():
                rospy.sleep(0.5)
                if self.plan_failed():
                    rospy.logerr("Plan failed! Robot may stuck in this place.")
                    continue
                print "Trigger crawl"
                with RobotCommander() as robot_commander:
                    robot_commander.crawl_trait()
                    rospy.sleep(0.1)
            
            # up_stair
            if (
                des_point.getPreTaskPoint()
                and des_point.getPreTaskPoint().is_up_stair()
            ):
                rospy.sleep(0.5)
                if self.plan_failed():
                    rospy.logerr("Plan failed! Robot may stuck in this place.")
                    continue
                print "Trigger up stair"
                with RobotCommander() as robot_commander:
                    robot_commander.up_stair_trait()
                    rospy.sleep(0.1)

            done = self.moveBaseClient.wait_for_result(timeout=rospy.Duration(5.0))
            # Make sure the action succeed.
            not_done = (not done) or (
                self.moveBaseClient.get_state() != actionlib.GoalStatus.SUCCEEDED
            )
            # print "Goal status: ", self.moveBaseClient.get_state() == actionlib.GoalStatus.SUCCEEDED

        """
        Do something to finish the special action
        """
        # finish up_stair
        if (
            not self.plan_failed()
            and des_point.getPreTaskPoint()
            and des_point.getPreTaskPoint().is_up_stair()
        ):
            print "Finish up stair"
            with RobotCommander() as robot_commander:
                robot_commander.finish_up_stair_trait()
                rospy.sleep(0.1)
            rospy.sleep(0.5)
        
        # finish crawl
        if (
            not self.plan_failed()
            and des_point.getPreTaskPoint()
            and des_point.getPreTaskPoint().is_crawl()
        ):
            print "Finish crawl"
            with RobotCommander() as robot_commander:
                robot_commander.finish_crawl_trait()
                rospy.sleep(0.1)
            rospy.sleep(0.5)
        
        # others
        if (
            not self.plan_failed()
            and des_point.getPreTaskPoint()
            and des_point.getPreTaskPoint().is_down_stair()
        ):
            print "Finish Others"
            with RobotCommander() as robot_commander:
                robot_commander.motion_start_stop()
                rospy.sleep(0.1)
            rospy.sleep(0.5)
        
        
