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

from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

import rclpy
from rclpy.logging import get_logger

from sciurus17_examples_py.utils import plan_and_execute


class NeckControl:
    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.sciurus17 = MoveItPy(node_name='neck_control')
        self.logger = get_logger('neck_control')

        # 首制御用 planning component
        self.neck = self.sciurus17.get_planning_component('neck_group')

        # planning scene monitor（現在のジョイント角度取得に使用）
        self.planning_scene_monitor = self.sciurus17.get_planning_scene_monitor()

        # ロボットモデルの取得（ジョイント目標値の設定に使用）
        self.robot_model = self.sciurus17.get_robot_model()

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.neck_plan_params = PlanRequestParameters(self.sciurus17, 'ompl_rrtc_default')
        self.neck_plan_params.max_velocity_scaling_factor = 0.1  # Set 0.0 ~ 1.0
        self.neck_plan_params.max_acceleration_scaling_factor = 0.1  # Set 0.0 ~ 1.0

    def move_to_named_pose(self, configuration_name):
        # SRDFに定義された姿勢名で首を動かす
        self.neck.set_start_state_to_current_state()
        self.neck.set_goal_state(configuration_name=configuration_name)
        plan_and_execute(
            self.sciurus17, self.neck, self.logger,
            single_plan_parameters=self.neck_plan_params,
        )

    def move_joint_values(self, joint_values):
        # ジョイント角度[rad]のリストを指定して首を動かす
        self.neck.set_start_state_to_current_state()
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('neck_group', joint_values)
        self.neck.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.sciurus17, self.neck, self.logger,
            single_plan_parameters=self.neck_plan_params,
        )

    def get_current_joint_values(self):
        # 首の現在のジョイント角度をリスト形式で取得する
        joint_values = []
        with self.planning_scene_monitor.read_only() as scene:
            robot_state = scene.current_state
            joint_values = robot_state.get_joint_group_positions('neck_group')
        return joint_values


def main(args=None):
    rclpy.init(args=args)

    controller = NeckControl()

    # 初期姿勢に移動
    controller.move_to_named_pose('neck_init_pose')

    # 現在の首のジョイント角度を取得
    joint_values = controller.get_current_joint_values()

    # 首を左に向ける
    joint_values[0] = math.radians(45.0)
    controller.move_joint_values(joint_values)

    # 首を右に向ける
    joint_values[0] = math.radians(-45.0)
    controller.move_joint_values(joint_values)

    # 首を前に向ける
    joint_values[0] = math.radians(0.0)
    controller.move_joint_values(joint_values)

    # 首を上に向ける
    joint_values[1] = math.radians(45.0)
    controller.move_joint_values(joint_values)

    # 首を下に向ける
    joint_values[1] = math.radians(-45.0)
    controller.move_joint_values(joint_values)

    # 初期姿勢に戻す
    controller.move_to_named_pose('neck_init_pose')

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
