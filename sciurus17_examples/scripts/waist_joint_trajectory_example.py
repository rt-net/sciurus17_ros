#!/usr/bin/env python
# coding: utf-8

# Copyright 2018 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import JointTrajectoryPoint
import sys
import math

class WaistYaw(object):
    # 初期化処理
    def __init__(self):
        self.__client = actionlib.SimpleActionClient("/sciurus17/controller3/waist_yaw_controller/follow_joint_trajectory",
                                                     FollowJointTrajectoryAction)
        self.__client.wait_for_server(rospy.Duration(5.0))

        if not self.__client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.present_angle = 0.0

    # 現在角度を設定
    def set_present_angle(self, angle):
        self.present_angle = angle

    # 目標角度の設定と実行
    def set_angle(self, yaw_angle, goal_sec):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["waist_yaw_joint"]

        # 現在の角度から開始（遷移時間は現在時刻）
        yawpoint = JointTrajectoryPoint()
        yawpoint.positions.append(self.present_angle)
        yawpoint.time_from_start = rospy.Duration(nsecs=1)
        goal.trajectory.points.append(yawpoint)

        # 途中に角度と時刻をappendすると細かい速度制御が可能
        # 参考=> http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement

        # ゴール角度を設定（指定されたゴール時間で到達）
        yawpoint = JointTrajectoryPoint()
        yawpoint.positions.append(yaw_angle)
        yawpoint.time_from_start = rospy.Duration(goal_sec)
        goal.trajectory.points.append(yawpoint)
        self.present_angle = yaw_angle

        # 軌道計画を実行
        self.__client.send_goal(goal)
        self.__client.wait_for_result(rospy.Duration(goal_sec))
        return self.__client.get_result()

if __name__ == '__main__':
    rospy.init_node("waist_test")

    wy = WaistYaw()
    wy.set_present_angle(math.radians(0.0))

    # まず正面を向く
    wy.set_angle(math.radians(0.0), 0.1)
    rospy.sleep(1.0)
    # 左45度,1秒
    wy.set_angle(math.radians(45.0), 1.0)
    rospy.sleep(1.0)
    # 正面：1秒
    wy.set_angle(math.radians(0.0), 1.0)
    rospy.sleep(1.0)
    # 右45度,1秒
    wy.set_angle(math.radians(-45.0), 1.0)
    rospy.sleep(1.0)
    # 正面,1秒
    wy.set_angle(math.radians(0.0), 1.0)
