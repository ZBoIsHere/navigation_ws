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
from pipeline.msg import MoveToPosition2DAction, MoveToPosition2DGoal
from pipeline.msg import MoveBaseAction, MoveBaseGoal
from TaskPoint import TaskPoint
from RobotCommander import RobotCommander


class TaskTransfer:
    def __init__(self):
        self.moveToClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.moveToClient.wait_for_server()
        rospy.loginfo("Action 'move_base' is up!")

    def plan_failed(self):
        return self.moveToClient.get_state() == actionlib.GoalStatus.ABORTED

    def is_action_succeed(self):
        return self.moveToClient.get_state() == actionlib.GoalStatus.SUCCEEDED

    def task_transfer(self, src_point, des_point):
        '''
        任务逻辑决策函数
        '''
        with RobotCommander() as robot_commander:
            robot_commander.sendCordinate(
                command_code=51,
                x=src_point.getPosX(),
                y=src_point.getPosY(),
                yaw=src_point.getYaw()
            )
            print ("sendCordinate")
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

        print "Option:"
        print des_point.record["option"]

        not_done = True

        while not_done and not rospy.is_shutdown():
            '''
            执行两点间的任务，一直到成功或失败
            '''
            self.moveToClient.send_goal(goal_msg)
            rospy.logwarn(
                "Transfer ftom [%s] to [%s]" % (src_point.name, des_point.name)
            )

            # crawl
            if des_point.getPreTaskPoint() and des_point.getPreTaskPoint().is_crawl():
                rospy.sleep(0.5)
                if self.plan_failed():
                    rospy.logerr("Plan failed! Robot may stuck in this place.")
                    continue
                print "crawl"
                with RobotCommander() as robot_commander:
                    robot_commander.crawl_trait()
                    rospy.sleep(0.1)
            # openDoor?
            if (
                des_point.getPreTaskPoint()
                and des_point.getPreTaskPoint().is_openDoor()
            ):
                rospy.sleep(0.5)
                if self.plan_failed():
                    rospy.logerr("Plan failed! Robot may stuck in this place.")
                    continue
                print "openDoor"
                with RobotCommander() as robot_commander:
                    robot_commander.openDoor_trait()
                    rospy.sleep(0.1)
            # leaveDoor?
            if (
                des_point.getPreTaskPoint()
                and des_point.getPreTaskPoint().is_leaveDoor()
            ):
                rospy.sleep(0.5)
                if self.plan_failed():
                    rospy.logerr("Plan failed! Robot may stuck in this place.")
                    continue
                print "leaveDoor"
                with RobotCommander() as robot_commander:
                    robot_commander.leaveDoor_trait()
                    rospy.sleep(0.1)
            # walking_bricks
            if (
                des_point.getPreTaskPoint()
                and des_point.getPreTaskPoint().is_walking_bricks()
            ):
                rospy.sleep(0.5)
                if self.plan_failed():
                    rospy.logerr("Plan failed! Robot may stuck in this place.")
                    continue
                print "walking_bricks"
                with RobotCommander() as robot_commander:
                    robot_commander.walking_bricks_trait()
                    rospy.sleep(0.1)
            # down_stair
            if (
                des_point.getPreTaskPoint()
                and des_point.getPreTaskPoint().is_down_stair()
            ):
                rospy.sleep(0.5)
                if self.plan_failed():
                    rospy.logerr("Plan failed! Robot may stuck in this place.")
                    continue
                print "Down stair trait"
                with RobotCommander() as robot_commander:
                    robot_commander.down_stair_trait()
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
                print "Up stair trait"
                with RobotCommander() as robot_commander:
                    robot_commander.up_stair_trait()
                    rospy.sleep(0.1)
            
            done = self.moveToClient.wait_for_result(timeout=rospy.Duration(300.0))
            
            # Make sure the action succeed.
            not_done = (not done) or (
                self.moveToClient.get_state() != actionlib.GoalStatus.SUCCEEDED
            )
            print "Goal status: ", self.moveToClient.get_state() == actionlib.GoalStatus.SUCCEEDED
        '''
        两点间的任务执行完成
        '''
        # 到达任务点后结束下楼梯
        if (
            not self.plan_failed()
            and des_point.getPreTaskPoint()
            and des_point.getPreTaskPoint().is_down_stair()
        ):
            print "Finish Down stair trait"
            with RobotCommander() as robot_commander:
                robot_commander.finish_down_stair_trait()
                rospy.sleep(0.1)
            rospy.sleep(0.5)
        # 到达任务点后结束上楼梯
        if (
            not self.plan_failed()
            and des_point.getPreTaskPoint()
            and des_point.getPreTaskPoint().is_up_stair()
        ):
            print "Finish Up stair trait"
            with RobotCommander() as robot_commander:
                robot_commander.finish_up_stair_trait()
                rospy.sleep(0.1)
            rospy.sleep(0.5)
        # 到达任务点后用延时的方式上斜坡
        if not self.plan_failed() and des_point.is_climbSlope():
            rospy.sleep(0.5)
            if self.plan_failed():
                rospy.logerr("Plan failed! Robot may stuck in this place.")
                print "climbSlope"
            with RobotCommander() as robot_commander:
                robot_commander.climbSlope_trait()
                rospy.sleep(18)
        
        if not self.plan_failed() and des_point.is_climbSlope():
            print "Finish climbSlope"
            with RobotCommander() as robot_commander:
                robot_commander.finish_climbSlope_trait()
                rospy.sleep(0.1)
            rospy.sleep(0.5)
        
        if (
            not self.plan_failed()
            and des_point.getPreTaskPoint()
            and des_point.getPreTaskPoint().is_crawl()
        ):
            print "Finish Crawl"
            with RobotCommander() as robot_commander:
                robot_commander.finish_crawl_trait()
                rospy.sleep(0.1)
            rospy.sleep(0.5)

        if (
            not self.plan_failed()
            and des_point.getPreTaskPoint()
            and des_point.getPreTaskPoint().is_walking_bricks()
        ):
            print "Finish walking_bricks"
            with RobotCommander() as robot_commander:
                robot_commander.finish_walking_bricks_trait()
                rospy.sleep(0.1)
            rospy.sleep(0.5)
