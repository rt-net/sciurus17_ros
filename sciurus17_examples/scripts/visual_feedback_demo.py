#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rosnode
import actionlib
from tf.transformations import quaternion_from_euler
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from ar_track_alvar_msgs.msg import AlvarMarkers
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import JointTrajectoryPoint
import tf
import sys
import math
import os
import copy


class VisualFeedbacker(object):
    def __init__(self):
        self._start_registering_time = 0.0
        self._last_seen_marker_time = -100
        self._ar_relative_rot = 0.0
        self._marker_id = -1
        self._marker_pos = ()
        self._marker_ori = ()

        self._ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self._callback_get_marker_pos)
        self._frame_listener = tf.TransformListener()
        self._frame_br = tf.TransformBroadcaster()

    def _callback_get_marker_pos(self, markers):
        # ARマーカー登録中に一番最初のマーカーを登録する
        if rospy.get_rostime().secs - self._start_registering_time <= 1.5:
            self._marker_id = markers.markers[0].id

        self._last_seen_marker_time = rospy.get_rostime().secs
        
        for m in markers.markers:
            if self._marker_id == m.id:
                self._marker_pos = (m.pose.pose.position.x,
                                    m.pose.pose.position.y,
                                    m.pose.pose.position.z)
                self._marker_ori = (m.pose.pose.orientation.x,
                                    m.pose.pose.orientation.y,
                                    m.pose.pose.orientation.z,
                                    m.pose.pose.orientation.w)

    def _check_available_marker(self):
        if rospy.get_rostime().secs - self._last_seen_marker_time > 1.0:
            rospy.logerr("ERROR: THERE IS NO AR MARKER")
            self._terminate()
        else:
            return True

    def debug_hand_position(self, target_pose):
        if self._check_available_marker():
            ar_relative_pos = self._get_ar_marker_transform()
            pos_diff_x =  target_pose.position.x - (self._marker_pos[0] + ar_relative_pos[0])
            pos_diff_y =  target_pose.position.y - (self._marker_pos[1] + ar_relative_pos[1])
            pos_diff_z =  target_pose.position.z - (self._marker_pos[2] + ar_relative_pos[2])
            
            return [pos_diff_x, pos_diff_y, pos_diff_z]

    def _get_ar_marker_transform(self):
        try:
            ar_trans, ar_rot= self._frame_listener.lookupTransform("base_link", "r_link5_armarker", rospy.Time(0))
            hand_trans, hand_rot = self._frame_listener.lookupTransform("base_link", "r_link7", rospy.Time(0))

            return (hand_trans[0] - ar_trans[0], 
                    hand_trans[1] - ar_trans[1], 
                    hand_trans[2] - ar_trans[2])
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("ERROR: COULDN'T GET AR MARKER TRANSFORMATION")
            rospy.logerr(str(e))
            self._terminate()

    def calculate_feedback_pos(self, target_position, hand_diff):
        MIN_TRHESHOLD = 0.0004 # meter
        MAX_TRHESHOLD_Z = 0.0008 # meter
        feedback_gain = [0.6, 0.6, 2.0, 0.01]
        feedback_position = copy.deepcopy(target_position)

        # 0.4mm以下の誤差は位置を補正しない
        for i in range(3): 
            if abs(hand_diff[i]) <= MIN_TRHESHOLD: 
                feedback_gain[i] = 0.0

        feedback_position.x += hand_diff * feedback_gain[0]
        feedback_position.y += hand_diff * feedback_gain[1]
        
        # z軸方向で0.8mm以下の誤差は位置の発散を防ぐためにより小さいゲインを使用する
        if (feedback_gain[2] != 0.0) and (MIN_TRHESHOLD < abs(hand_diff[2]) < MAX_TRHESHOLD_Z):
            feedback_position.z += hand_diff * feedback_gain[3]
        else:
            feedback_position.z += hand_diff * feedback_gain[2]

        self._marker_pos = ()

        return feedback_position

    def register_ar_marker(self):
        rospy.loginfo("INITIALIZE ARMS POSITION FOR AR MARKER REGISTRATION")

        # ARマーカーが見えるようにカメラとハンドを動かすと
        neck_pitch.set_neck_angle(yaw= math.radians(-40.0), pitch= math.radians(-50.0))

        target_pose = geometry_msgs.msg.Pose() # ハンドの現実の目標位置
        target_pose.position.x = 0.3
        target_pose.position.y = -0.1
        target_pose.position.z = 0.3
        q = quaternion_from_euler(3.14/2.0, 0.0, 0.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        arm_controller.set_arm_pose(target_pose)

        # アームの先端のARマーカーを登録 
        self._start_registering_time = rospy.get_rostime().secs
        rospy.loginfo("START REGISTERING AR MARKER ID FOR RIGHT HAND")
        rospy.sleep(2.0)
       
        # ARマーカーがない場合はデモを終了
        if self._marker_id == -1:
            rospy.logerr("ERROR: THERE IS NO AR MARKER")
            self._terminate()

        arm_controller.initialize_arm_pose()
        rospy.loginfo("FINISHED REGISTERING AR MARKER ID FOR RIGHT HAND")

    def _terminate(self):
        ## SRDFに定義されている"home"の姿勢にする
        arm_controller.initialize_arm_pose()
        # 終了
        rospy.signal_shutdown('STOPPING')
        sys.exit(1)


class ArmController(object):
    def set_arm_pose(self, target_pose, sleep_time=1.5):
        rospy.loginfo("Move hand to position:" + str(target_pose))
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行
        rospy.sleep(sleep_time)

    def set_gripper_pose(self, gripper_pose):
        gripper_goal.command.position = gripper_pose
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(1.0))
    
    def initialize_arm_pose(self):
        arm.set_named_target("r_arm_init_pose")
        arm.go()
        self.set_gripper_pose(0.0)


class NeckPitch(object):
    def __init__(self):
        self.__client = actionlib.SimpleActionClient("/sciurus17/controller3/neck_controller/follow_joint_trajectory",
                                                     FollowJointTrajectoryAction)
        self.__client.wait_for_server(rospy.Duration(5.0))
        if not self.__client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)

    def set_neck_angle(self, yaw=0.0, pitch=0.0):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["neck_yaw_joint", "neck_pitch_joint"]

        yawpoint = JointTrajectoryPoint()
        yawpoint.positions.append(yaw)
        yawpoint.positions.append(pitch)
        yawpoint.time_from_start = rospy.Duration(nsecs=1)
        goal.trajectory.points.append(yawpoint)
        self.__client.send_goal(goal)
        self.__client.wait_for_result(rospy.Duration(0.1))

        return self.__client.get_result()


def main():
    # SRDFに定義されている"home"の姿勢にする
    os.system("clear")
    arm_controller.initialize_arm_pose()
    rospy.sleep(1.0)

    # ARマーカーを登録し始める
    visual_feedbacker.register_ar_marker()

    # デモを行う
    neck_pitch.set_neck_angle(yaw= math.radians(-10.0), pitch= math.radians(-65.0))

    reality_target_pose = geometry_msgs.msg.Pose() # ハンドの現実の目標位置
    reality_target_pose.position.x = 0.3
    reality_target_pose.position.y = 0.0
    reality_target_pose.position.z = 0.161+0.091 # z軸の値 = 三角形の高さ + 六角レンチの先端とハンドの先端間の距離
    q = quaternion_from_euler(3.14/2.0, 0.0, 0.0)
    reality_target_pose.orientation.x = q[0]
    reality_target_pose.orientation.y = q[1]
    reality_target_pose.orientation.z = q[2]
    reality_target_pose.orientation.w = q[3]

    moveit_target_pose = copy.deepcopy(reality_target_pose)

    # ARマーカーを使わないデモ
    arm_controller.set_arm_pose(reality_target_pose, 1.5)

    diff_before_feedback = visual_feedbacker.debug_hand_position(reality_target_pose)
    arm_controller.initialize_arm_pose()

    # ARマーカーでアームの先端位置を5回補正するデモ
    arm_controller.set_arm_pose(reality_target_pose, 1.5)

    for i in range(5):
        # ARマーカーを使って位置の誤差を計算する
        hand_diff = visual_feedbacker.debug_hand_position(reality_target_pose)
        feedbacked_position = visual_feedbacker.calculate_feedback_pos(moveit_target_pose.position, hand_diff)

        moveit_target_pose.position = copy.deepcopy(feedbacked_position)
        # 位置を補正し始める
        rospy.loginfo("START FIXING HAND POSITION WITH AR MARKER")
        arm_controller.set_arm_pose(moveit_target_pose, 1.0)

    diff_after_feedback = visual_feedbacker.debug_hand_position(reality_target_pose)
    
    print('\n')
    print("Position different before:", diff_before_feedback)
    print("Position different after:", diff_after_feedback)

    arm_controller.initialize_arm_pose()



if __name__ == "__main__":
    rospy.init_node("sciurus17_visual_feedback")

    neck_pitch = NeckPitch()
    arm_controller = ArmController()
    visual_feedbacker = VisualFeedbacker()

    # 右腕グループの制御
    arm = moveit_commander.MoveGroupCommander("r_arm_group")
    # 速度と加速度をゆっくりにする
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)
    # movitにおける位置と姿勢の誤差を減らす
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)

    # 右ハンド初期化
    gripper = actionlib.SimpleActionClient("/sciurus17/controller1/right_hand_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 2.0

    main()
