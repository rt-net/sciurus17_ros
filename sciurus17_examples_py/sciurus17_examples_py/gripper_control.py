# Copyright 2026 RT Corporation
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

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit.planning import PlanRequestParameters
import rclpy
from rclpy.logging import get_logger

from sciurus17_examples_py.utils import plan_and_execute


class GripperControl:
    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.sciurus17 = MoveItPy(node_name='gripper_control')
        self.logger = get_logger('gripper_control')

        # アーム・グリッパ制御用 planning component
        self.arm = self.sciurus17.get_planning_component('two_arm_group')
        self.l_gripper = self.sciurus17.get_planning_component(
            'l_gripper_group'
        )
        self.r_gripper = self.sciurus17.get_planning_component(
            'r_gripper_group'
        )

        # ロボットモデルの取得（ジョイント目標値の設定に使用）
        self.robot_model = self.sciurus17.get_robot_model()

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.arm_plan_params = PlanRequestParameters(
            self.sciurus17, 'ompl_rrtc_default'
        )
        # 0.0〜1.0の範囲で設定
        self.arm_plan_params.max_velocity_scaling_factor = 0.1 
        # 0.0〜1.0の範囲で設定
        self.arm_plan_params.max_acceleration_scaling_factor = 0.1

        self.gripper_plan_params = PlanRequestParameters(
            self.sciurus17, 'ompl_rrtc_default'
        )

    def move_arm_to_named_pose(self, configuration_name):
        # SRDFに定義された姿勢名でアームを動かす
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=configuration_name)
        plan_and_execute(
            self.sciurus17,
            self.arm,
            self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def move_r_gripper_angle(self, angle):
        # 右グリッパを角度[rad]を指定して開閉する
        self.r_gripper.set_start_state_to_current_state()
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('r_gripper_group', [angle])
        self.r_gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.sciurus17,
            self.r_gripper,
            self.logger,
            single_plan_parameters=self.gripper_plan_params,
        )

    def move_l_gripper_angle(self, angle):
        # 左グリッパを角度[rad]を指定して開閉する
        self.l_gripper.set_start_state_to_current_state()
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('l_gripper_group', [angle])
        self.l_gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.sciurus17,
            self.l_gripper,
            self.logger,
            single_plan_parameters=self.gripper_plan_params,
        )


def main(args=None):
    rclpy.init(args=args)

    controller = GripperControl()

    # グリッパの開閉角
    R_GRIPPER_CLOSE = math.radians(0.0)
    R_GRIPPER_OPEN = math.radians(40.0)
    L_GRIPPER_CLOSE = math.radians(0.0)
    L_GRIPPER_OPEN = math.radians(-40.0)

    # two_arm_init_poseの姿勢にする
    controller.move_arm_to_named_pose('two_arm_init_pose')

    # 右グリッパを2回開閉する
    for _ in range(2):
        controller.move_r_gripper_angle(R_GRIPPER_OPEN)
        controller.move_r_gripper_angle(R_GRIPPER_CLOSE)

    # 左グリッパを2回開閉する
    for _ in range(2):
        controller.move_l_gripper_angle(L_GRIPPER_OPEN)
        controller.move_l_gripper_angle(L_GRIPPER_CLOSE)

    # 既知の不具合により終了時にエラーになるが問題ない。関連Issue:
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
