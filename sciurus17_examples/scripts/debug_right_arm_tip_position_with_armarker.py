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
import pygame

#PS4コントローラのボタンのラベル
BUTTON_CROSS = 0
BUTTON_CIRCLE = 1
BUTTON_TRIANGLE = 2
BUTTON_SQUARE = 3

BUTTON_L1 = 4
BUTTON_R1 = 5
BUTTON_L2 = 6
BUTTON_R2 = 7
BUTTON_SHARE = 8
BUTTON_OPTIONS = 9
BUTTON_PS = 10

BUTTON_LEFT_STICK = 11
BUTTON_RIGHT_STICK = 12

HAT_1 = 0
HAT_RIGHT = (1,0)
HAT_LEFT = (-1,0)
HAT_UP = (0,1)
HAT_DOWN = (0,-1)

AXIS_LEFT_STICK_X = 0   
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 3
AXIS_RIGHT_STICK_Y = 4
AXIS_L2 = 2
AXIS_R2 = 5


class Sciurus(object):
    def __init__(self):
        self.ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.callback_get_armarker_position)
        self.__client = actionlib.SimpleActionClient("/sciurus17/controller3/neck_controller/follow_joint_trajectory",
                                                     FollowJointTrajectoryAction)
        self.__client.wait_for_server(rospy.Duration(5.0))

        if not self.__client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)

        self.yaw_angle = 0.0
        self.pitch_angle = 0.0
        self.marker_pose = None
        self.marker_orientation = None

    def callback_get_armarker_position(self, markers):
        for m in markers.markers:
            marker_id = m.id
            marker_pose = m.pose.pose
            self.marker_pose = pos = marker_pose.position
            self.marker_orientation = ori = marker_pose.orientation
            #rospy.loginfo("marker[%d] position (x, y, z) = (%3.3f: %3.3f: %3.3f), orientation (x, y, z, w) = (%3.3f: %3.3f: %3.3f)" %(marker_id, pos.x, pos.y, pos.z, ori.x, ori.z, ori.w)) 

    def get_armarker_postion(self):
        return self.marker_pose, self.marker_orientation

    def debug_arm_tip_with_ar_marker(self):
        ar_pose, ar_ori = self.get_armarker_postion()
        arm_pose = arm.get_current_pose().pose
        
        if ar_pose and ar_ori:
            self.output_message("DEBBUG HAND POSITION WITH AR MARKER")
            print "Position Different"
            print "X:", (arm_pose.position.x - ar_pose.x)*1000, "Y:", (arm_pose.position.y - ar_pose.y)*1000, "Z:", (arm_pose.position.z - ar_pose.z)*1000
            print "Orientation Different"
            print "X:", (arm_pose.orientation.x - ar_ori.x), "Y:", (arm_pose.orientation.y - ar_ori.y), "Z:", (arm_pose.orientation.z - ar_ori.z), "W:", (arm_pose.orientation.w - ar_ori.w)
        else:
            print "No AR MARKER FOUND"


    def set_arm_pose(self, target_pose, orientation_pose):
        arm_target_pose = geometry_msgs.msg.Pose()
        arm_target_pose.position.x = target_pose[0]
        arm_target_pose.position.y = target_pose[1]
        arm_target_pose.position.z = target_pose[2]
        q = quaternion_from_euler(orientation_pose[0], orientation_pose[1], orientation_pose[2])  # 上方から掴みに行く場合
        arm_target_pose.orientation.x = q[0]
        arm_target_pose.orientation.y = q[1]
        arm_target_pose.orientation.z = q[2]
        arm_target_pose.orientation.w = q[3]
        arm.set_pose_target(arm_target_pose)  # 目標ポーズ設定
        arm.go()  # 実行
        rospy.sleep(0.5)

    def set_gripper_pose(self, gripper_pose):
        gripper_goal.command.position = gripper_pose
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(2.0))

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

    def output_message(self, message):
        print "\n"
        print "="*15, message


class DualShock(object):
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        controller = pygame.joystick.Joystick(0)
        controller.init()
        
        self.axis = {}
        self.button = {}
        self.hat = {}

        for i in range(controller.get_numaxes()):
            self.axis[i] = 0.0
        for i in range(controller.get_numbuttons()):
            self.button[i] = 0.0
        for i in range(controller.get_numhats()):
            self.hat[i] = 0.0

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.axis[event.axis] = round(event.value,3)
            elif event.type == pygame.JOYBUTTONDOWN:
                self.button[event.button] = True
            elif event.type == pygame.JOYBUTTONUP:
                self.button[event.button] = False
            elif event.type == pygame.JOYHATMOTION:
                self.hat[event.hat] = event.value


def main():
    sc.output_message("Group names:")
    print robot.get_group_names()

    sc.output_message("Current state:")
    print robot.get_current_state()

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    sc.output_message("Arm initial pose:")
    print arm_initial_pose

    # SRDFに定義されている"home"の姿勢にする
    sc.initialize_arm_pose()

    # 体の前までに動かす
    sc.output_message("Move right hand to (0.25, 0.0, 0.3) position")
    sc.set_neck_angle(math.radians(-0.0), math.radians(-0.0))
    sc.set_arm_pose((0.3, 0.0, 0.3), (3.14/2.0, 0.0, 0.0))
    neck_yaw = neck_pitch = 0.0

    while not rospy.is_shutdown():
        ds4.update()

        pose = arm.get_current_pose().pose.position
        ori = arm.get_current_pose().pose.orientation
        #print pose, "\n"
        angle_y = target_x = target_y = target_z = 0.0

        if abs(ds4.axis[AXIS_LEFT_STICK_X]) > 0.9:
            target_x = 0.05 * ds4.axis[AXIS_LEFT_STICK_X]/abs(ds4.axis[AXIS_LEFT_STICK_X])
            ds4.axis[AXIS_LEFT_STICK_X] = 0.0
        if abs(ds4.axis[AXIS_LEFT_STICK_Y]) > 0.9:
            target_y = 0.05 * ds4.axis[AXIS_LEFT_STICK_Y]/abs(ds4.axis[AXIS_LEFT_STICK_Y])
            ds4.axis[AXIS_LEFT_STICK_Y] = 0.0
        if abs(ds4.axis[AXIS_RIGHT_STICK_Y]) > 0.9:
            target_z = 0.05 * ds4.axis[AXIS_RIGHT_STICK_Y]/abs(ds4.axis[AXIS_RIGHT_STICK_Y])
            ds4.axis[AXIS_RIGHT_STICK_Y] = 0.0

        if ds4.button[BUTTON_R1]:
            angle_y = 3.14/6
            ds4.button[BUTTON_R1] = False
        elif ds4.button[BUTTON_L1]:
            angle_y = -3.14/6
            ds4.button[BUTTON_L1] = False

        if ds4.hat[HAT_1]:
            neck_yaw -= ds4.hat[HAT_1][0] * 10
            neck_pitch += ds4.hat[HAT_1][1] * 10

            sc.set_neck_angle(math.radians(neck_yaw), math.radians(neck_pitch))
            ds4.hat[HAT_1] = 0

        if ds4.button[BUTTON_CIRCLE]:
            sc.debug_arm_tip_with_ar_marker() 

        if target_x or target_y or target_z or angle_y:
            print "\nmove arm"
            print target_x, target_y, target_z, angle_y
            if pose.z - target_z <= 0.2: target_z = 0.0
            sc.set_arm_pose((pose.x - target_y, pose.y - target_x, pose.z - target_z), (3.14/2.0, ori.y + angle_y, 0.0))

        r.sleep()

    # SRDFに定義されている"home"の姿勢にする
    sc.initialize_arm_pose()

    # 終了
    collision_object = moveit_msgs.msg.CollisionObject()
    moveit_commander.roscpp_shutdown()
    sc.output_message("STOPPING")


if __name__ == '__main__':
    rospy.init_node("sciurus17_debug_arm_tip_position_with_armarker")

    sc = Sciurus()
    robot = moveit_commander.RobotCommander()
    ds4 = DualShock()

    arm = moveit_commander.MoveGroupCommander("r_arm_group")
    gripper = actionlib.SimpleActionClient("/sciurus17/controller1/right_hand_controller/gripper_cmd", GripperCommandAction)
    gripper_goal = GripperCommandGoal()
    r = rospy.Rate(20)

    arm.set_max_velocity_scaling_factor(0.5)
    gripper.wait_for_server()
    gripper_goal.command.max_effort = 2.0


    rospy.sleep(1.0)

    main()
