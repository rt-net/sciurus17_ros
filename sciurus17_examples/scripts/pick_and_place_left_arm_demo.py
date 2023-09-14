#! /usr/bin/env python
# -*- coding: utf-8 -*-

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
import moveit_commander
import geometry_msgs.msg
import rosnode
import actionlib
from tf.transformations import quaternion_from_euler
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)


def main():
    rospy.init_node("sciurus17_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("l_arm_group")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = actionlib.SimpleActionClient("/sciurus17/controller2/left_hand_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 2.0

    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    gripper_goal.command.position = -0.9
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("l_arm_init_pose")
    arm.go()
    gripper_goal.command.position = 0.0
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))

    # 掴む準備をする
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.25
    target_pose.position.y = 0.0
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを開く
    gripper_goal.command.position = -0.7
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))

    # 掴みに行く
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.25
    target_pose.position.y = 0.0
    target_pose.position.z = 0.13
    q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを閉じる
    gripper_goal.command.position = -0.4
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))

    # 持ち上げる
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.25
    target_pose.position.y = 0.0
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()							# 実行

    # 移動する
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.4
    target_pose.position.y = 0.0
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # 下ろす
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.4
    target_pose.position.y = 0.0
    target_pose.position.z = 0.13
    q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを開く
    gripper_goal.command.position = -0.7
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))

    # 少しだけハンドを持ち上げる
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.4
    target_pose.position.y = 0.0
    target_pose.position.z = 0.2
    q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("l_arm_init_pose")
    arm.go()

    print("done")


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
