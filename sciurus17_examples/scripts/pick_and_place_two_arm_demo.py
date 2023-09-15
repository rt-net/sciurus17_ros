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
from moveit_msgs.msg import Constraints, OrientationConstraint

def main():
    rospy.init_node("sciurus17_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()

    eef_constraints = Constraints()
    l_eef_oc = OrientationConstraint()
    r_eef_oc = OrientationConstraint()

    # 両腕グループの制御
    arm = moveit_commander.MoveGroupCommander("two_arm_group")
    # 速度と加速度をゆっくりにする
    arm.set_max_velocity_scaling_factor(0.2)
    arm.set_max_acceleration_scaling_factor(0.2)
    # 右ハンド初期化
    r_gripper = actionlib.SimpleActionClient("/sciurus17/controller1/right_hand_controller/gripper_cmd", GripperCommandAction)
    r_gripper.wait_for_server()
    r_gripper_goal = GripperCommandGoal()
    r_gripper_goal.command.max_effort = 2.0
    # 左ハンド初期化
    l_gripper = actionlib.SimpleActionClient("/sciurus17/controller2/left_hand_controller/gripper_cmd", GripperCommandAction)
    l_gripper.wait_for_server()
    l_gripper_goal = GripperCommandGoal()
    l_gripper_goal.command.max_effort = 2.0

    rospy.sleep(1.0)

    # グループリスト表示
    print("Group names:")
    print(robot.get_group_names())

    # ステータス表示
    print("Current state:")
    print(robot.get_current_state())

    # 右アーム初期ポーズを表示
    r_arm_initial_pose = arm.get_current_pose("r_link7").pose
    print("Right arm initial pose:")
    print(r_arm_initial_pose)
    # 左アーム初期ポーズを表示
    l_arm_initial_pose = arm.get_current_pose("l_link7").pose
    print("Left arm initial pose:")
    print(l_arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    # 右手を開く
    r_gripper_goal.command.position = 0.9
    r_gripper.send_goal(r_gripper_goal)
    r_gripper.wait_for_result(rospy.Duration(1.0))
    # 左手を開く
    l_gripper_goal.command.position = -0.9
    l_gripper.send_goal(l_gripper_goal)
    l_gripper.wait_for_result(rospy.Duration(1.0))

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("two_arm_init_pose")
    arm.go()
    # 右手を閉じる
    r_gripper_goal.command.position = 0.0
    r_gripper.send_goal(r_gripper_goal)
    r_gripper.wait_for_result(rospy.Duration(1.0))
    # 左手を閉じる
    l_gripper_goal.command.position = 0.0
    l_gripper.send_goal(l_gripper_goal)
    l_gripper.wait_for_result(rospy.Duration(1.0))

    # 掴む準備をする
    # 右腕
    r_target_pose = geometry_msgs.msg.Pose()
    r_target_pose.position.x = 0.18
    r_target_pose.position.y = -0.25
    r_target_pose.position.z = 0.2
    q = quaternion_from_euler(3.14/2.0, 0.0, 0.0)  # まずは手を下に向ける
    r_target_pose.orientation.x = q[0]
    r_target_pose.orientation.y = q[1]
    r_target_pose.orientation.z = q[2]
    r_target_pose.orientation.w = q[3]
    # 左腕
    l_target_pose = geometry_msgs.msg.Pose()
    l_target_pose.position.x = 0.18
    l_target_pose.position.y = 0.25
    l_target_pose.position.z = 0.2
    q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # まずは手を下に向ける
    l_target_pose.orientation.x = q[0]
    l_target_pose.orientation.y = q[1]
    l_target_pose.orientation.z = q[2]
    l_target_pose.orientation.w = q[3]
    # 目標ポーズ設定
    arm.set_pose_target(r_target_pose,"r_link7")
    arm.set_pose_target(l_target_pose,"l_link7")
    arm.go()  # 実行

    # ハンドを開く
    # 右手を開く
    r_gripper_goal.command.position = 0.7
    r_gripper.send_goal(r_gripper_goal)
    r_gripper.wait_for_result(rospy.Duration(1.0))
    # 左手を開く
    l_gripper_goal.command.position = -0.7
    l_gripper.send_goal(l_gripper_goal)
    l_gripper.wait_for_result(rospy.Duration(1.0))

    # 手を内向きにする
    # 右腕
    r_target_pose = geometry_msgs.msg.Pose()
    r_target_pose.position.x = 0.18
    r_target_pose.position.y = -0.25
    r_target_pose.position.z = 0.08
    q = quaternion_from_euler(3.14, 0.0, 0.0) # 手を内向きにする
    r_target_pose.orientation.x = q[0]
    r_target_pose.orientation.y = q[1]
    r_target_pose.orientation.z = q[2]
    r_target_pose.orientation.w = q[3]
    # 左腕
    l_target_pose = geometry_msgs.msg.Pose()
    l_target_pose.position.x = 0.18
    l_target_pose.position.y = 0.25
    l_target_pose.position.z = 0.08
    q = quaternion_from_euler(-3.14, 0.0, 0.0) # 手を内向きにする
    l_target_pose.orientation.x = q[0]
    l_target_pose.orientation.y = q[1]
    l_target_pose.orientation.z = q[2]
    l_target_pose.orientation.w = q[3]
    # 目標ポーズ設定
    arm.set_pose_target(r_target_pose,"r_link7")
    arm.set_pose_target(l_target_pose,"l_link7")
    arm.go()  # 実行

    # ハンドを開く
    # 右手を開く
    r_gripper_goal.command.position = 1.0
    r_gripper.send_goal(r_gripper_goal)
    r_gripper.wait_for_result(rospy.Duration(1.0))
    # 左手を開く
    l_gripper_goal.command.position = -1.0
    l_gripper.send_goal(l_gripper_goal)
    l_gripper.wait_for_result(rospy.Duration(1.0))

    # 掴みに行く準備
    # 右腕
    r_target_pose = geometry_msgs.msg.Pose()
    r_target_pose.position.x = 0.18
    r_target_pose.position.y = -0.10
    r_target_pose.position.z = 0.08
    q = quaternion_from_euler(3.14, 0.0, 0.0)
    r_target_pose.orientation.x = q[0]
    r_target_pose.orientation.y = q[1]
    r_target_pose.orientation.z = q[2]
    r_target_pose.orientation.w = q[3]
    # 左腕
    l_target_pose = geometry_msgs.msg.Pose()
    l_target_pose.position.x = 0.18
    l_target_pose.position.y = 0.10
    l_target_pose.position.z = 0.08
    q = quaternion_from_euler(-3.14, 0.0, 0.0)
    l_target_pose.orientation.x = q[0]
    l_target_pose.orientation.y = q[1]
    l_target_pose.orientation.z = q[2]
    l_target_pose.orientation.w = q[3]
    # 軌道制約の追加
    # 右手を平行に寄せる
    r_eef_pose = arm.get_current_pose("r_link7").pose
    r_eef_oc.header.stamp = rospy.Time.now()
    r_eef_oc.header.frame_id = "/base_link"
    r_eef_oc.link_name ="r_link7"
    r_eef_oc.orientation = r_eef_pose.orientation
    r_eef_oc.absolute_x_axis_tolerance = 1.5708
    r_eef_oc.absolute_y_axis_tolerance = 0.4
    r_eef_oc.absolute_z_axis_tolerance = 0.7
    r_eef_oc.weight = 1
    # 左手を平行に寄せる
    l_eef_pose = arm.get_current_pose("l_link7").pose
    l_eef_oc.header.stamp = rospy.Time.now()
    l_eef_oc.header.frame_id = "/base_link"
    l_eef_oc.link_name ="l_link7"
    l_eef_oc.orientation = l_eef_pose.orientation
    l_eef_oc.absolute_x_axis_tolerance = 1.5708
    l_eef_oc.absolute_y_axis_tolerance = 0.4
    l_eef_oc.absolute_z_axis_tolerance = 0.7
    l_eef_oc.weight = 1
    eef_constraints.orientation_constraints.append(r_eef_oc)
    eef_constraints.orientation_constraints.append(l_eef_oc)
    arm.set_path_constraints(eef_constraints)
    # 目標ポーズ設定
    arm.set_pose_target(r_target_pose,"r_link7")
    arm.set_pose_target(l_target_pose,"l_link7")
    arm.go()  # 実行

    # 掴みに行く
    # 右腕
    r_target_pose = geometry_msgs.msg.Pose()
    r_target_pose.position.x = 0.18
    r_target_pose.position.y = -0.08
    r_target_pose.position.z = 0.08
    q = quaternion_from_euler(3.14, 0.0, 0.0)
    r_target_pose.orientation.x = q[0]
    r_target_pose.orientation.y = q[1]
    r_target_pose.orientation.z = q[2]
    r_target_pose.orientation.w = q[3]
    # 左腕
    l_target_pose = geometry_msgs.msg.Pose()
    l_target_pose.position.x = 0.18
    l_target_pose.position.y = 0.08
    l_target_pose.position.z = 0.08
    q = quaternion_from_euler(-3.14, 0.0, 0.0)
    l_target_pose.orientation.x = q[0]
    l_target_pose.orientation.y = q[1]
    l_target_pose.orientation.z = q[2]
    l_target_pose.orientation.w = q[3]
    # 目標ポーズ設定
    arm.set_pose_target(r_target_pose,"r_link7")
    arm.set_pose_target(l_target_pose,"l_link7")
    arm.go()  # 実行

    # ハンドを閉じる
    # 右手を閉じる
    r_gripper_goal.command.position = 0.4
    r_gripper.send_goal(r_gripper_goal)
    r_gripper.wait_for_result(rospy.Duration(1.0))
    # 左手を閉じる
    l_gripper_goal.command.position = -0.4
    l_gripper.send_goal(l_gripper_goal)
    l_gripper.wait_for_result(rospy.Duration(1.0))

    # 前へ押し出す
    # 右腕
    r_target_pose = geometry_msgs.msg.Pose()
    r_target_pose.position.x = 0.35
    r_target_pose.position.y = -0.09
    r_target_pose.position.z = 0.08
    q = quaternion_from_euler(3.14, 0.0, 0.0)
    r_target_pose.orientation.x = q[0]
    r_target_pose.orientation.y = q[1]
    r_target_pose.orientation.z = q[2]
    r_target_pose.orientation.w = q[3]
    # 左腕
    l_target_pose = geometry_msgs.msg.Pose()
    l_target_pose.position.x = 0.35
    l_target_pose.position.y = 0.09
    l_target_pose.position.z = 0.08
    q = quaternion_from_euler(-3.14, 0.0, 0.0)
    l_target_pose.orientation.x = q[0]
    l_target_pose.orientation.y = q[1]
    l_target_pose.orientation.z = q[2]
    l_target_pose.orientation.w = q[3]
    # 目標ポーズ設定
    arm.set_pose_target(r_target_pose,"r_link7")
    arm.set_pose_target(l_target_pose,"l_link7")
    arm.go()  # 実行

    # ハンドを開く
    # 右手を開く
    r_gripper_goal.command.position = 1.5
    r_gripper.send_goal(r_gripper_goal)
    r_gripper.wait_for_result(rospy.Duration(1.0))
    # 左手を開く
    l_gripper_goal.command.position = -1.5
    l_gripper.send_goal(l_gripper_goal)
    l_gripper.wait_for_result(rospy.Duration(1.0))

    # 軌道制約の解除
    arm.set_path_constraints(None)

    # 1秒待つ
    rospy.sleep(1.0)

    # 離す
    # 右腕
    r_target_pose = geometry_msgs.msg.Pose()
    r_target_pose.position.x = 0.28
    r_target_pose.position.y = -0.20
    r_target_pose.position.z = 0.12
    q = quaternion_from_euler(3.14/2.0, 0.0, 0.0)  # 手を下向きにして逃がす
    r_target_pose.orientation.x = q[0]
    r_target_pose.orientation.y = q[1]
    r_target_pose.orientation.z = q[2]
    r_target_pose.orientation.w = q[3]
    # 左腕
    l_target_pose = geometry_msgs.msg.Pose()
    l_target_pose.position.x = 0.28
    l_target_pose.position.y = 0.20
    l_target_pose.position.z = 0.12
    q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 手を下向きにして逃がす
    l_target_pose.orientation.x = q[0]
    l_target_pose.orientation.y = q[1]
    l_target_pose.orientation.z = q[2]
    l_target_pose.orientation.w = q[3]
    # 目標ポーズ設定
    arm.set_pose_target(r_target_pose,"r_link7")
    arm.set_pose_target(l_target_pose,"l_link7")
    arm.go()  # 実行

    # ハンドを閉じる
    # 右手を閉じる
    r_gripper_goal.command.position = 0.4
    r_gripper.send_goal(r_gripper_goal)
    r_gripper.wait_for_result(rospy.Duration(1.0))
    # 左手を閉じる
    l_gripper_goal.command.position = -0.4
    l_gripper.send_goal(l_gripper_goal)
    l_gripper.wait_for_result(rospy.Duration(1.0))

    # 腕を持ち上げてターゲットから離れる
    # 右腕
    r_target_pose = geometry_msgs.msg.Pose()
    r_target_pose.position.x = 0.32
    r_target_pose.position.y = -0.25
    r_target_pose.position.z = 0.30
    q = quaternion_from_euler(3.14/2.0, -3.14/2.0, 0.0)
    r_target_pose.orientation.x = q[0]
    r_target_pose.orientation.y = q[1]
    r_target_pose.orientation.z = q[2]
    r_target_pose.orientation.w = q[3]
    # 左腕
    l_target_pose = geometry_msgs.msg.Pose()
    l_target_pose.position.x = 0.32
    l_target_pose.position.y = 0.25
    l_target_pose.position.z = 0.30
    q = quaternion_from_euler(-3.14/2.0, -3.14/2.0, 0.0)
    l_target_pose.orientation.x = q[0]
    l_target_pose.orientation.y = q[1]
    l_target_pose.orientation.z = q[2]
    l_target_pose.orientation.w = q[3]
    # 目標ポーズ設定
    arm.set_pose_target(r_target_pose,"r_link7")
    arm.set_pose_target(l_target_pose,"l_link7")
    arm.go()  # 実行

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("two_arm_init_pose")
    arm.go()

    print("done")


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
