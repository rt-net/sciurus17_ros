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
import sys
import math


class Sciurus(object):
    def __init__(self):
        self.yaw_angle = 0.0
        self.pitch_angle = 0.0
        self.marker_poses = {}
        self.marker_list = {}
        self.last_seen_marker = 0.0
        
        self.ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.callback_get_marker_pos)
        self.__client = actionlib.SimpleActionClient("/sciurus17/controller3/neck_controller/follow_joint_trajectory",
                                                     FollowJointTrajectoryAction)
        self.__client.wait_for_server(rospy.Duration(5.0))
        
        if not self.__client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)


    def callback_get_marker_pos(self, markers):
        self.last_seen_marker = rospy.get_rostime().secs
        
        # 検出された1つまたは複数のマーカーを記憶する
        for m in markers.markers:
            marker_id = m.id
            marker_pose = m.pose.pose
            
            if marker_id not in self.marker_poses.keys():
                self.marker_poses[marker_id] = [marker_pose]
            else:
                self.marker_poses[marker_id].append(marker_pose)
            
            # それぞれのARマーカーを10回記憶する
            if len(self.marker_poses[marker_id]) > 10: 
                del(self.marker_poses[marker_id][0])
            
            # ARマーカーの位置の急な変化を防ぐために平均値を記憶する
            self.marker_list = self.get_armarker_avg_pos()
            
            #rospy.loginfo("marker[%d] position (x, y, z) = (%3.3f: %3.3f: %3.3f), orientation (x, y, z, w) = (%3.3f: %3.3f: %3.3f)" %(marker_id, pos.x, pos.y, pos.z, ori.x, ori.z, ori.w)) 


    def remove_old_marker(self):
        if rospy.get_rostime().secs - self.last_seen_marker > 0.5:
            for m_list in self.marker_poses.values():
                if len(m_list): del(m_list[0])


    def get_armarker_avg_pos(self):
        marker_poses_list = {}
        for m_id, m_list in self.marker_poses.items():
            if len(m_list) > 5:
                pos_x = pos_y = pos_z = ori_x = ori_y = ori_z = ori_w = 0.0
                for m in m_list[-1::-1]: # 新しいマーカーのデータ順から計算する
                    pos_x += m.position.x
                    pos_y += m.position.y 
                    pos_z += m.position.z 
                    ori_x += m.orientation.x
                    ori_y += m.orientation.y
                    ori_z += m.orientation.z
                    ori_w += m.orientation.w
                marker_poses_list[m_id] = [sum_val/len(m_list) for sum_val in [pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w]]
        return marker_poses_list


    def debug_hand_position(self, target_pose):
        m_id = self.marker_id
        
        pos_diff_x =  target_pose[0] - self.marker_list[m_id][0]
        pos_diff_y =  target_pose[1] - self.marker_list[m_id][1]
        pos_diff_z =  target_pose[2] - self.marker_list[m_id][2]
        
        return [pos_diff_x, pos_diff_y, pos_diff_z]


    def fix_arm_tip_with_ar_marker(self, target_pos, target_ori):
        if self.marker_list:
            hand = arm.get_current_pose().pose
            
            self.output_message("START FIXING HAND POSITION WITH AR MARKER")
            
            hand_diff = self.debug_hand_position(target_pos)
            print(hand_diff)
            for i, val in enumerate(hand_diff): 
                if abs(val) <= 0.01: hand_diff[i] = 0.0 # 10mm以下の誤差は位置の補正を止める
            
            pos_x = hand.position.x + hand_diff[0] *0.8
            pos_y = hand.position.y + hand_diff[1] *0.8
            pos_z = hand.position.z

            if hand_diff[2] != 0: 
                if hand_diff[2] > 0.0:
                    pos_z = hand.position.z + hand_diff[2] +0.01
                else: 
                    pos_z = hand.position.z + hand_diff[2] -0.004
            
            # x, y, z軸に沿って位置を補正する
            sc.set_arm_pose((pos_x, hand.position.y, hand.position.z), target_ori, 0.8)
            sc.set_arm_pose((pos_x, pos_y, hand.position.z), target_ori, 0.8)
            sc.set_arm_pose((pos_x, pos_y, pos_z), target_ori, 0.8)
        else:
            rospy.logwarn("CANNOT DEBUG HAND TIP BECAUSE NO AR MARKER FOUND")


    def set_arm_pose(self, target_pose, orientation_pose, sleep_time=1.5):
        arm_target_pose = geometry_msgs.msg.Pose()
        arm_target_pose.position.x = target_pose[0]
        arm_target_pose.position.y = target_pose[1]
        arm_target_pose.position.z = target_pose[2]
        
        q = quaternion_from_euler(orientation_pose[0], orientation_pose[1], orientation_pose[2])  # 上方から掴みに行く場合
        arm_target_pose.orientation.x = q[0]
        arm_target_pose.orientation.y = q[1]
        arm_target_pose.orientation.z = q[2]
        arm_target_pose.orientation.w = q[3]
        
        self.output_message("Move hand to position:" + str(target_pose))
        arm.set_pose_target(arm_target_pose)  # 目標ポーズ設定
        arm.go()  # 実行
        rospy.sleep(sleep_time)


    def set_gripper_pose(self, gripper_pose):
        gripper_goal.command.position = gripper_pose
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(1.0))

    
    def initialize_arm_pose(self):
        self.set_neck_angle(math.radians(0.0), math.radians(0.0))
        arm.set_named_target("r_arm_init_pose")
        arm.go()
        self.set_gripper_pose(0.0)


    def set_neck_angle(self, yaw_angle, pitch_angle):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["neck_yaw_joint", "neck_pitch_joint"]
        yawpoint = JointTrajectoryPoint()
        self.yaw_angle = yaw_angle
        self.pitch_angle = pitch_angle
        yawpoint.positions.append(self.yaw_angle)
        yawpoint.positions.append(self.pitch_angle)
        yawpoint.time_from_start = rospy.Duration(nsecs=1)
        goal.trajectory.points.append(yawpoint)
        self.__client.send_goal(goal)
        self.__client.wait_for_result(rospy.Duration(0.1))
        return self.__client.get_result()


    def register_ar_marker(self):
        # 両手の位置を初期化
        neck_yaw = -40.0
        neck_pitch = -50.0
        self.set_neck_angle(math.radians(neck_yaw), math.radians(neck_pitch))
        
        self.output_message("INITIALIZE ARMS POSITION FOR AR MARKER REGISTRATION")
        pos = (0.3, -0.1, 0.3)
        ori = (3.14/2.0, 0.0, 0.0)
        self.set_arm_pose(pos, ori)
        rospy.sleep(2.0)
        
        self.output_message("REGISTERING AR MARKER ID FOR RIGHT HAND")
        rospy.sleep(2.0)
        
        # アームの先端のARマーカーを登録 
        if len(self.marker_list) > 0:
            self.marker_id = self.marker_list.keys()[0]
        else:
            rospy.logerr("ERROR: THERE IS NOT ENOUGH AR MARKER")
            self.terminate()
        self.initialize_arm_pose()


    def terminate(self):
        ## SRDFに定義されている"home"の姿勢にする
        self.initialize_arm_pose()
        
        # 終了
        collision_object = moveit_msgs.msg.CollisionObject()
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("STOPPING")
        sys.exit(1)


    def output_message(self, message):
        print "\n"
        rospy.loginfo("*"*30)
        rospy.loginfo(message)
        rospy.loginfo("*"*30)


def main():
    # SRDFに定義されている"home"の姿勢にする
    sc.initialize_arm_pose()
    rospy.sleep(1.0)

    # ARマーカーを登録し始める
    sc.register_ar_marker()

    # VISUAL_FEEDBACKの実験を始める
    neck_yaw = 0.0
    neck_pitch = -60.0
    sc.set_neck_angle(math.radians(neck_yaw), math.radians(neck_pitch))

    # ハンドを何回か動かす事によって、目標位置の誤差が生まれる
    pos = (0.3, -0.2, 0.3)
    ori = (3.14/2.0, 0.0, 0.0)
    sc.set_arm_pose(pos, ori)
    sc.set_gripper_pose(0.5)

    pos = (0.3, -0.2, 0.12)
    ori = (3.14/2.0, 0.0, 0.0)
    sc.set_arm_pose(pos, ori)
    sc.set_gripper_pose(0.0)
    
    pos = (0.3, -0.2, 0.3)
    ori = (3.14/2.0, 0.0, 0.0)
    sc.set_arm_pose(pos, ori)

    pos = (0.3, 0.0, 0.3)
    ori = (3.14/2.0, 0.0, 0.0)
    sc.set_arm_pose(pos, ori)

    # ARマーカーでアームの先端位置を5回補正する
    for i in range(5):
        sc.fix_arm_tip_with_ar_marker(pos, ori)
        sc.remove_old_marker() # 記憶の古いマーカーを忘れる

    while not rospy.is_shutdown():
        diff = sc.debug_hand_position(pos)
        sc.output_message("HAND POSITION DIFF "+ str(diff))
        rospy.sleep(1.0)

    sc.terminate()


if __name__ == "__main__":
    rospy.init_node("sciurus17_visual_feedback_with_a_pen")

    sc = Sciurus()
    robot = moveit_commander.RobotCommander()

    # 両腕グループの制御
    arm = moveit_commander.MoveGroupCommander("r_arm_group")
    # 速度と加速度をゆっくりにする
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)

    # 右ハンド初期化
    gripper = actionlib.SimpleActionClient("/sciurus17/controller1/right_hand_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 2.0

    r = rospy.Rate(20)

    rospy.sleep(1.0)
    
    # 実験を始める
    main()

