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


class VisualFeedbacker(object):
    def __init__(self):
        self._ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.callback_get_marker_pos)
        self.start_registering_time = 0.0
        self.last_seen_marker_time = 0.0
        self.marker_id = -1
        self.marker_pos = ()

    def callback_get_marker_pos(self, markers):
        # ARマーカー登録中に一番最初のマーカーを登録する
        #print("stam:", markers.header.stamp)
        if rospy.get_rostime().secs - self.start_registering_time <= 1.5:
            self.marker_id = markers.markers[0].id

        #self.last_seen_marker_time = markers.header.stamp
        self.last_seen_marker_time = rospy.get_rostime().secs
        
        for m in markers.markers:
            if self.marker_id == m.id:
                self.marker_pos = (m.pose.pose.position.x,
                                   m.pose.pose.position.y,
                                   m.pose.pose.position.z)

    def check_available_marker(self):
        if rospy.get_rostime().secs - self.last_seen_marker_time > 0.5:
            rospy.logerr("ERROR: THERE IS NO AR MARKER")
            self.terminate()
        else:
            return True

    def debug_hand_position(self, target_pose):
        pos_diff_x =  target_pose[0] - self.marker_pos[0]
        pos_diff_y =  target_pose[1] - self.marker_pos[1]
        pos_diff_z =  target_pose[2] - self.marker_pos[2]
        
        return [pos_diff_x, pos_diff_y, pos_diff_z]

    def calculate_feedback_pos(self, target_pos, current_pos):
        if self.check_available_marker():
            rospy.loginfo("START FIXING HAND POSITION WITH AR MARKER")
            
            hand_diff = self.debug_hand_position(target_pos)
            for i, val in enumerate(hand_diff): 
                if abs(val) <= 0.005: hand_diff[i] = 0.0 # 5mm以下の誤差は位置を補正しない
            
            pos_x = current_pos.position.x + hand_diff[0] *0.7
            pos_y = current_pos.position.y + hand_diff[1] *0.7
            pos_z = current_pos.position.z + hand_diff[2] *0.04 + 0.005

            print('hand diff:', hand_diff)
            print('marker_pose', self.marker_pos)

            self.marker_pos = ()

            return pos_x, pos_y, pos_z

    def register_ar_marker(self):
        rospy.loginfo("INITIALIZE ARMS POSITION FOR AR MARKER REGISTRATION")

        # ARマーカーが見えるようにカメラとハンドを動かすと
        np.set_neck_angle(yaw= math.radians(-40.0), pitch= math.radians(-50.0))
        ac.set_arm_pose(pos= (0.3, -0.15, 0.3), ori= (3.14/2.0, 0.0, 0.0))

        # アームの先端のARマーカーを登録 
        self.start_registering_time = rospy.get_rostime().secs
        rospy.loginfo("REGISTERING AR MARKER ID FOR RIGHT HAND")
        rospy.sleep(2.0)
       
        # ARマーカーがない場合はデモを終了
        if self.marker_id == -1:
            rospy.logerr("ERROR: THERE IS NO AR MARKER")
            self.terminate()

        ac.initialize_arm_pose()

    def terminate(self):
        ## SRDFに定義されている"home"の姿勢にする
        ac.initialize_arm_pose()
        
        # 終了
        collision_object = moveit_msgs.msg.CollisionObject()
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown('STOPPING')
        sys.exit(1)


class ArmController(object):
    def set_arm_pose(self, pos=(0.0, 0.0, 0.0), 
                           ori=(0.0, 0.0, 0.0), 
                           sleep_time=1.5):
        arm_target_pose = geometry_msgs.msg.Pose()
        arm_target_pose.position.x = pos[0]
        arm_target_pose.position.y = pos[1]
        arm_target_pose.position.z = pos[2]
        
        q = quaternion_from_euler(ori[0], ori[1], ori[2])
        arm_target_pose.orientation.x = q[0]
        arm_target_pose.orientation.y = q[1]
        arm_target_pose.orientation.z = q[2]
        arm_target_pose.orientation.w = q[3]
        
        rospy.loginfo("Move hand to position:" + str(pos))
        arm.set_pose_target(arm_target_pose)  # 目標ポーズ設定
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
    ac.initialize_arm_pose()
    rospy.sleep(1.0)

    # ARマーカーを登録し始める
    vf.register_ar_marker()

    # デモを行う
    np.set_neck_angle(yaw= math.radians(0.0), pitch= math.radians(-60.0))
    target_pos = (0.3, 0.0, 0.3)
    target_ori = (3.14/2.0, 0.0, 0.0)


    # ARマーカーを使わないデモ
    ac.set_arm_pose(target_pos, target_ori)
    diff_before_feedback = vf.debug_hand_position(target_pos)
    ac.initialize_arm_pose()


    # ARマーカーでアームの先端位置を5回補正するデモ
    ac.set_arm_pose(target_pos, target_ori)

    for i in range(5):
        current_pos = arm.get_current_pose().pose

        diff_x, diff_y, diff_z = vf.calculate_feedback_pos(target_pos, current_pos)

        # x, y, z軸に沿って位置を補正する
        ac.set_arm_pose((diff_x, current_pos.position.y, current_pos.position.z), target_ori, 1.0)
        ac.set_arm_pose((diff_x, diff_y, current_pos.position.z), target_ori, 1.0)
        ac.set_arm_pose((diff_x, diff_y, diff_z), target_ori, 1.0)

    diff_after_feedback = vf.debug_hand_position(target_pos)
    
    print('\n')
    print("Position different before:", diff_before_feedback)
    print("Position different after:", diff_after_feedback)

    rospy.sleep(1.0)


if __name__ == "__main__":
    '''
    デモを行う場所
    '''
    rospy.init_node("sciurus17_visual_feedback")

    np = NeckPitch()
    ac = ArmController()
    vf = VisualFeedbacker()

    # 右腕グループの制御
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
    
    main()

