#! /usr/bin/env python
# coding: utf-8

# Copyright 2019 RT Corporation
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
import math
import sys

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose

# for NeckYawPitch
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTrajectoryControllerState
)
from trajectory_msgs.msg import JointTrajectoryPoint

# for Stacker
import moveit_commander
import actionlib
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


class NeckYawPitch(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient("/sciurus17/controller3/neck_controller/follow_joint_trajectory",
                                                     FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration(5.0))
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)


    def set_angle(self, yaw_angle, pitch_angle, goal_secs=1.0e-9):
        # 首を指定角度に動かす
        # Move the neck to the designated angle.
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["neck_yaw_joint", "neck_pitch_joint"]

        yawpoint = JointTrajectoryPoint()
        yawpoint.positions.append(yaw_angle)
        yawpoint.positions.append(pitch_angle)
        yawpoint.time_from_start = rospy.Duration(goal_secs)
        goal.trajectory.points.append(yawpoint)

        self._client.send_goal(goal)
        self._client.wait_for_result(rospy.Duration(0.1))
        return self._client.get_result()


class Stacker(object):
    _RIGHT_ARM = 1
    _LEFT_ARM = 2
    # グリッパの開閉角度（radians）
    # Gripper opening and closing angles (radians)
    _GRIPPER_OPEN = 0.8
    _GRIPPER_CLOSE = 0.42

    def __init__(self):
        self._markers = MarkerArray()

        self._marker_sub = rospy.Subscriber("/sciurus17/example/markers",
                MarkerArray, self._markers_callback, queue_size=1)

        self._right_arm = moveit_commander.MoveGroupCommander("r_arm_waist_group")
        self._right_arm.set_max_velocity_scaling_factor(0.1)
        self._right_gripper = actionlib.SimpleActionClient("/sciurus17/controller1/right_hand_controller/gripper_cmd", GripperCommandAction)
        self._right_gripper.wait_for_server()

        self._left_arm = moveit_commander.MoveGroupCommander("l_arm_waist_group")
        self._left_arm.set_max_velocity_scaling_factor(0.1)
        self._left_gripper = actionlib.SimpleActionClient("/sciurus17/controller2/left_hand_controller/gripper_cmd", GripperCommandAction)
        self._left_gripper.wait_for_server()

        self._gripper_goal = GripperCommandGoal()
        self._gripper_goal.command.max_effort = 2.0

        # アームとグリッパー姿勢の初期化
        # Initialize arm and gripper pose
        self.initialize_arms()

        self._current_arm = None


    def _markers_callback(self, msg):
        self._markers = msg

    def get_num_of_markers(self):
        return len(self._markers.markers)

    def _get_lowest_object_pose(self):
        # 一番高さが低いオブジェクトのPoseを返す
        # Return the Pose of the object with the lowest height.

        lowest_z = 1000 
        lowest_pose = Pose()

        for marker in self._markers.markers:
            if marker.pose.position.z < lowest_z:
                # marker.pose.position は箱の中心座標を表す
                lowest_pose = marker.pose
                lowest_z = marker.pose.position.z
                lowest_pose.position.z += marker.scale.z * 0.5 # 箱の大きさ分高さを加算する

        return lowest_pose

    def _get_highest_object_pose(self):
        # 一番高さが高いオブジェクトのPoseを返す
        # Return the Pose of the object with the highest height

        highest_z = 0
        highest_pose = Pose()

        for marker in self._markers.markers:
            if marker.pose.position.z > highest_z:
                # marker.pose.position は箱の中心座標を表す
                # marker.pose.position represents the box centre coordinates
                highest_pose = marker.pose
                highest_z = marker.pose.position.z
                # 箱の大きさ分高さを加算する
                # Adds the box size to the height.
                highest_pose.position.z += marker.scale.z * 0.5

        return highest_pose


    def _move_arm(self, current_arm, target_pose):
        if current_arm == self._RIGHT_ARM:
            # 手先を下に向ける
            # Pointing the hand tip downwards.
            q = quaternion_from_euler(3.14/2.0, 0.0, 0.0)
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            self._right_arm.set_pose_target(target_pose)
            return self._right_arm.go()
        elif current_arm == self._LEFT_ARM:
            # 手先を下に向ける
            # Pointing the hand tip downwards.
            q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            self._left_arm.set_pose_target(target_pose)
            return self._left_arm.go()
        else:
            return False


    def _move_arm_to_init_pose(self, current_arm):
        if current_arm == self._RIGHT_ARM:
            self._right_arm.set_named_target("r_arm_waist_init_pose")
            return self._right_arm.go()
        elif current_arm == self._LEFT_ARM:
            self._left_arm.set_named_target("l_arm_waist_init_pose")
            return self._left_arm.go()
        else:
            return False


    def _open_gripper(self, current_arm):
        if current_arm == self._RIGHT_ARM:
            self._gripper_goal.command.position = self._GRIPPER_OPEN
            self._right_gripper.send_goal(self._gripper_goal)
            return self._right_gripper.wait_for_result(rospy.Duration(1.0))
        elif current_arm == self._LEFT_ARM:
            self._gripper_goal.command.position = -self._GRIPPER_OPEN
            self._left_gripper.send_goal(self._gripper_goal)
            return self._left_gripper.wait_for_result(rospy.Duration(1.0))
        else:
            return False


    def _close_gripper(self, current_arm):
        if current_arm == self._RIGHT_ARM:
            self._gripper_goal.command.position = self._GRIPPER_CLOSE
            self._right_gripper.send_goal(self._gripper_goal)
        elif current_arm == self._LEFT_ARM:
            self._gripper_goal.command.position = -self._GRIPPER_CLOSE
            self._left_gripper.send_goal(self._gripper_goal)
        else:
            return False


    def initialize_arms(self):
        self._move_arm_to_init_pose(self._RIGHT_ARM)
        self._move_arm_to_init_pose(self._LEFT_ARM)
        self._open_gripper(self._RIGHT_ARM)
        self._open_gripper(self._LEFT_ARM)


    def pick_up(self, check_result):
        # 一番高さが低いオブジェクトを持ち上げる
        # Lift the object with the lowest height.
        rospy.loginfo("PICK UP")
        result = True
        # 制御対象を初期化
        # Initialize control target.
        self._current_arm = None

        # オブジェクトがなければ終了
        # Exit if no object
        rospy.sleep(1.0)
        if self.get_num_of_markers() == 0:
            rospy.logwarn("NO OBJECTS")
            return False

        object_pose = self._get_lowest_object_pose()

        # オブジェクトの位置によって左右のどちらの手で取るかを判定する
        # Determine whether the object should be taken with the left or right hand depending on its position.
        if object_pose.position.y < 0:
            self._current_arm = self._RIGHT_ARM
            rospy.loginfo("Set right arm")
        else:
            self._current_arm = self._LEFT_ARM
            rospy.loginfo("Set left arm")

        self._open_gripper(self._current_arm)

        # Z軸方向のオフセット meters
        #  Offset in Z-axis (meters)
        APPROACH_OFFSET = 0.10
        PREPARE_OFFSET = 0.06
        LEAVE_OFFSET = 0.10

        # 目標手先姿勢の生成
        # Generate target hand pose
        target_pose = Pose()
        target_pose.position.x = object_pose.position.x
        target_pose.position.y = object_pose.position.y
        target_pose.position.z = object_pose.position.z

        # 掴む準備をする
        # Prepare to grasp
        target_pose.position.z = object_pose.position.z + APPROACH_OFFSET
        if self._move_arm(self._current_arm, target_pose) is False and check_result:
            rospy.logwarn("Approach failed")
            result = False

        else:
            rospy.sleep(1.0)
            # ハンドを下げる
            # Lower the hand
            target_pose.position.z = object_pose.position.z + PREPARE_OFFSET
            if self._move_arm(self._current_arm, target_pose) is False and check_result:
                rospy.logwarn("Preparation grasping failed")
                result = False

            else:
                rospy.sleep(1.0)
                # つかむ
                # Grasp
                if self._close_gripper(self._current_arm) is False and check_result:
                    rospy.logwarn("Grasping failed")
                    result = False

                rospy.sleep(1.0)
                # ハンドを上げる
                # Raise the hand
                target_pose.position.z = object_pose.position.z + LEAVE_OFFSET
                self._move_arm(self._current_arm, target_pose)


        if result is False:
            rospy.sleep(1.0)
            # 失敗したときは安全のため手を広げる
            # Open the hand for safety if it fails
            self._open_gripper(self._current_arm)

        rospy.sleep(1.0)
        # 初期位置に戻る
        # Return to initial position
        self._move_arm_to_init_pose(self._current_arm)

        return result


    def place_on(self, check_result, target_x=0.3, target_y=0):
        # 座標x,yにオブジェクトを配置する
        # Place the object at coordinates x,y
        rospy.loginfo("PLACE on :" + str(target_x) + ", " + str(target_y))
        result = True

        # 制御アームが設定されているかチェック
        # Check if the control arm is set
        if self._current_arm is None:
            rospy.logwarn("NO ARM SELECTED")
            return False

        # Z軸方向のオフセット meters
        # Offset in Z-axis direction meters.
        APPROACH_OFFSET = 0.14
        PREPARE_OFFSET = 0.10
        LEAVE_OFFSET = 0.14

        # 目標手先姿勢の生成
        # Generate target hand pose
        target_pose = Pose()
        target_pose.position.x = target_x
        target_pose.position.y = target_y
        target_pose.position.z = 0.0

        # 置く準備をする
        # Prepare to place
        target_pose.position.z = APPROACH_OFFSET
        if self._move_arm(self._current_arm, target_pose) is False and check_result:
            rospy.logwarn("Approach failed")
            result = False
        else:
            rospy.sleep(1.0)
            # ハンドを下げる
            # Lower the hand
            target_pose.position.z = PREPARE_OFFSET
            if self._move_arm(self._current_arm, target_pose) is False and check_result:
                rospy.logwarn("Preparation release failed")
                result = False
            else:
                rospy.sleep(1.0)
                # はなす
                # Release
                self._open_gripper(self._current_arm)
                # ハンドを上げる
                # Raise the hand
                target_pose.position.z = LEAVE_OFFSET
                self._move_arm(self._current_arm, target_pose)

        if result is False:
            rospy.sleep(1.0)
            # 失敗したときは安全のため手を広げる
            # Open the hand for safety if it fails to move the arm.
            self._open_gripper(self._current_arm)

        rospy.sleep(1.0)
        # 初期位置に戻る
        # Return to initial position
        self._move_arm_to_init_pose(self._current_arm)

        return result


    def place_on_highest_object(self, check_result):
        # 一番高さが高いオブジェクトの上に配置する
        # Place on top of the highest object
        rospy.loginfo("PLACE ON HIGHEST OBJECT")
        result = True

        # 制御アームが設定されているかチェック
        # Check if the control arm is set
        if self._current_arm is None:
            rospy.logwarn("NO ARM SELECTED")
            return False

        # オブジェクトが他になければデフォルト位置に置く
        # Place the object in the default position if no other object exists
        rospy.sleep(1.0)
        if self.get_num_of_markers() == 0:
            rospy.logwarn("NO OBJECTS")
            return self.place_on(check_result)

        object_pose = self._get_highest_object_pose()

        # Z軸方向のオフセット meters
        # Offset in Z-axis direction meters
        APPROACH_OFFSET = 0.15
        PREPARE_OFFSET = 0.11
        LEAVE_OFFSET = 0.15

        # 目標手先姿勢の生成
        # Generate target hand pose
        target_pose = Pose()
        target_pose.position.x = object_pose.position.x
        target_pose.position.y = object_pose.position.y
        target_pose.position.z = object_pose.position.z

        # 置く準備をする
        # Prepare to place
        target_pose.position.z = object_pose.position.z + APPROACH_OFFSET
        if self._move_arm(self._current_arm, target_pose) is False and check_result:
            rospy.logwarn("Approach failed")
            result = False
        else:
            rospy.sleep(1.0)
            # ハンドを下げる
            # Lower the hand
            target_pose.position.z = object_pose.position.z + PREPARE_OFFSET
            if self._move_arm(self._current_arm, target_pose) is False and check_result:
                rospy.logwarn("Preparation release failed")
                result = False
            else:
                rospy.sleep(1.0)
                # はなす
                # Release
                self._open_gripper(self._current_arm)
                # ハンドを上げる
                # Raise the hand
                target_pose.position.z = object_pose.position.z + LEAVE_OFFSET
                self._move_arm(self._current_arm, target_pose)

        if result is False:
            rospy.sleep(1.0)
            # 失敗したときは安全のため手を広げる
            # Open the hand for safety if it fails to move the arm.
            self._open_gripper(self._current_arm)

        rospy.sleep(1.0)
        # 初期位置に戻る
        # Return to initial position
        self._move_arm_to_init_pose(self._current_arm)

        return result


def hook_shutdown():
    # 首の角度を戻す
    # Move the neck to 0 degrees
    neck.set_angle(math.radians(0), math.radians(0), 3.0)
    # 両腕を初期位置に戻す
    # Move both arms to initial position
    stacker.initialize_arms()


def main():
    r = rospy.Rate(60)

    rospy.on_shutdown(hook_shutdown)

    # 首の初期角度 degree
    # Initial angle of neck. degree
    INITIAL_YAW_ANGLE = 0
    INITIAL_PITCH_ANGLE = -70
    # 首を下に傾けてテーブル上のものを見る
    # Tilt neck down to see objects on the table
    neck.set_angle(math.radians(INITIAL_YAW_ANGLE), math.radians(INITIAL_PITCH_ANGLE))

    PICKING_MODE = 0
    PLACE_MODE = 1
    current_mode = PICKING_MODE

    # 箱の配置位置 meter
    # Box placement position. meter
    PLACE_X = 0.3
    PLACE_Y = 0.0

    # アームとグリッパが正しく動いたかチェックする
    # Check if arm and gripper moved correctly
    CHECK_RESULT = True

    while not rospy.is_shutdown():
        if current_mode == PICKING_MODE:
            if stacker.get_num_of_markers() > 0:
                # マーカがあれば、１番高さが低いオブジェクトを掴み上げる
                # If there is a marker, pick up the object with the lowest height.
                if stacker.pick_up(CHECK_RESULT) is False:
                    rospy.logwarn("PickUp Failed")
                else:
                    rospy.loginfo("PickUp Succeeded")
                    current_mode = PLACE_MODE
            else:
                rospy.loginfo("NO MARKERS")
                rospy.sleep(1.0)
        
        elif current_mode == PLACE_MODE:
            if stacker.get_num_of_markers() == 0:
                # マーカがなければ、指定位置に配置
                # If there is no marker, place it at the specified position
                if stacker.place_on(CHECK_RESULT, PLACE_X, PLACE_Y) is False:
                    rospy.logwarn("Place Failed")
                else:
                    rospy.loginfo("Place Succeeded")
        
            else:
                # マーカがあれば、一番高い位置にあるオブジェクトの上に配置
                # If there is a marker, place it on top of the highest object
                if stacker.place_on_highest_object(CHECK_RESULT) is False:
                    rospy.logwarn("Place Failed")
                else:
                    rospy.loginfo("Place Succeeded")
        
            current_mode = PICKING_MODE

        r.sleep()


if __name__ == '__main__':
    rospy.init_node("box_stacking_example")

    neck = NeckYawPitch()
    stacker = Stacker()

    main()

