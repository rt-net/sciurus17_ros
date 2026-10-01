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

from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit.planning import PlanRequestParameters
import rclpy
from rclpy.logging import get_logger

from sciurus17_examples_py.utils import plan_and_execute


class JointValues:
    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.sciurus17 = MoveItPy(node_name='joint_values')
        self.logger = get_logger('joint_values')

        # アーム制御用 planning component
        self.arm = self.sciurus17.get_planning_component('l_arm_group')

        # ロボットモデルの取得（ジョイント目標値の設定に使用）
        self.robot_model = self.sciurus17.get_robot_model()

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.arm_plan_params = PlanRequestParameters(
            self.sciurus17, 'ompl_rrtc_default'
        )
        self.arm_plan_params.max_velocity_scaling_factor = 0.1  # 0.0〜1.0の範囲で設定
        self.arm_plan_params.max_acceleration_scaling_factor = (
            0.1  # 0.0〜1.0の範囲で設定
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

    def move_arm_joint_values(self, joint_values_dict):
        # 各ジョイント角度[rad]を指定してアームを動かす
        # joint_values_dictはジョイント名をキー、角度[rad]を値とする辞書
        robot_state = RobotState(self.robot_model)
        robot_state.joint_positions = joint_values_dict

        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=self.robot_model.get_joint_model_group(
                'l_arm_group'
            ),
        )

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(motion_plan_constraints=[joint_constraint])
        plan_and_execute(
            self.sciurus17,
            self.arm,
            self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def get_current_arm_joint_values(self):
        # アームの現在のジョイント角度を辞書形式で取得する
        current_state = self.arm.get_start_state()
        return current_state.get_joint_group_positions('l_arm_group')


def main(args=None):
    rclpy.init(args=args)

    controller = JointValues()

    joint_names = [
        'l_arm_joint1',
        'l_arm_joint2',
        'l_arm_joint3',
        'l_arm_joint4',
        'l_arm_joint5',
        'l_arm_joint6',
        'l_arm_joint7',
    ]
    target_joint_diff_value = math.radians(15.0)

    # l_arm_init_poseの姿勢にする
    controller.move_arm_to_named_pose('l_arm_init_pose')

    # 各関節角度を初期姿勢から順番に15[deg]ずつ動かす
    joint_values = controller.get_current_arm_joint_values()
    joint_values_dict = dict(zip(joint_names, joint_values))

    for joint_index, joint_name in enumerate(joint_names):
        if joint_values[joint_index] > 0.1:
            joint_values_dict[joint_name] -= target_joint_diff_value
        else:
            joint_values_dict[joint_name] += target_joint_diff_value
        controller.move_arm_joint_values(joint_values_dict)

    # l_arm_init_poseの姿勢に戻す
    controller.move_arm_to_named_pose('l_arm_init_pose')

    # 既知の不具合により終了時にエラーになるが問題ない。関連Issue:
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
