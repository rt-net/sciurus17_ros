# Copyright 2024 RT Corporation
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

import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit.planning import PlanRequestParameters
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
import rclpy
from rclpy.logging import get_logger

from sciurus17_examples_py.utils import plan_and_execute


class PickAndPlace:
    # グリッパの開閉角度
    GRIPPER_OPEN = math.radians(40.0)
    GRIPPER_GRASP = math.radians(20.0)
    GRIPPER_CLOSE = math.radians(0.0)

    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.sciurus17 = MoveItPy(node_name='pick_and_place_right_arm_waist')
        self.logger = get_logger('pick_and_place_right_arm_waist')

        # 右腕・腰・右グリッパ制御用 planning component
        self.r_arm_waist_group = self.sciurus17.get_planning_component(
            'r_arm_waist_group'
        )
        self.r_gripper_group = self.sciurus17.get_planning_component(
            'r_gripper_group'
        )

        # ロボットモデルの取得（ジョイント目標値の設定に使用）
        self.robot_model = self.sciurus17.get_robot_model()

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.arm_plan_params = PlanRequestParameters(
            self.sciurus17, 'ompl_rrtc_default'
        )
        self.arm_plan_params.max_velocity_scaling_factor = 0.1  # Set 0.0 ~ 1.0
        self.arm_plan_params.max_acceleration_scaling_factor = (
            0.1  # Set 0.0 ~ 1.0
        )

        self.gripper_plan_params = PlanRequestParameters(
            self.sciurus17, 'ompl_rrtc_default'
        )

    def move_arm_to_pose(self, pose):
        # アームを目標位置・姿勢（Pose）に動かす
        # 座標系はbase_link、目標リンクはr_link7
        self.r_arm_waist_group.set_start_state_to_current_state()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.pose = pose
        self.r_arm_waist_group.set_goal_state(
            pose_stamped_msg=goal_pose,
            pose_link='r_link7',
        )
        plan_and_execute(
            self.sciurus17,
            self.r_arm_waist_group,
            self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def control_arm(self, x, y, z):
        # アームを目標位置（x, y, z [m]）に動かす（姿勢は下向き固定）
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = 0.707
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.707
        self.move_arm_to_pose(pose)

    def move_arm_to_named_pose(self, configuration_name):
        # SRDFに定義された姿勢名でアームを動かす
        self.r_arm_waist_group.set_start_state_to_current_state()
        self.r_arm_waist_group.set_goal_state(
            configuration_name=configuration_name
        )
        plan_and_execute(
            self.sciurus17,
            self.r_arm_waist_group,
            self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def move_gripper_angle(self, angle):
        # グリッパを角度[rad]を指定して開閉する
        self.r_gripper_group.set_start_state_to_current_state()
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('r_gripper_group', [angle])
        self.r_gripper_group.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.sciurus17,
            self.r_gripper_group,
            self.logger,
            single_plan_parameters=self.gripper_plan_params,
        )

    def set_constraints(self):
        # 腰関節の可動範囲制限を設定する
        constraints = Constraints()
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'waist_yaw_joint'
        joint_constraint.position = 0.0
        joint_constraint.tolerance_above = math.radians(45.0)
        joint_constraint.tolerance_below = math.radians(45.0)
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
        self.r_arm_waist_group.set_path_constraints(
            path_constraints=constraints
        )

    def clear_constraints(self):
        # 設定された関節可動制限をクリアする
        self.r_arm_waist_group.set_path_constraints(path_constraints=None)


def main(args=None):
    rclpy.init(args=args)

    controller = PickAndPlace()

    # アプローチ・退避時の高さ
    LIFTING_HEIGHT = 0.25

    # 掴む位置（ピック位置）のXYZ[m]
    PICK_X = 0.25
    PICK_Y = 0.0
    PICK_Z = 0.12

    # 置く位置（プレース位置）のXYZ[m]
    PLACE_X = 0.35
    PLACE_Y = 0.0
    PLACE_Z = 0.12

    # 初期化動作
    controller.move_arm_to_named_pose('r_arm_waist_init_pose')
    controller.move_gripper_angle(controller.GRIPPER_OPEN)
    controller.set_constraints()

    # ピック動作（掴みに行く）
    # 物体の上に腕を伸ばす
    controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT)
    # アプローチ
    controller.control_arm(PICK_X, PICK_Y, PICK_Z)
    # 掴む
    controller.move_gripper_angle(controller.GRIPPER_GRASP)
    # 持ち上げる
    controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT)

    # プレース動作（移動して置く）
    # 移動する
    controller.control_arm(PLACE_X, PLACE_Y, LIFTING_HEIGHT)
    # 下ろす
    controller.control_arm(PLACE_X, PLACE_Y, PLACE_Z)
    # 離す
    controller.move_gripper_angle(controller.GRIPPER_OPEN)
    # 持ち上げる
    controller.control_arm(PLACE_X, PLACE_Y, LIFTING_HEIGHT)

    # 終了動作
    controller.clear_constraints()
    controller.move_arm_to_named_pose('r_arm_waist_init_pose')
    controller.move_gripper_angle(controller.GRIPPER_CLOSE)

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
