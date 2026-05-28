# Copyright 2025 RT Corporation
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

from sciurus17_examples_py.utils import plan_and_execute

from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

import rclpy
from rclpy.logging import get_logger


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('joint_values')

    # instantiate MoveItPy instance and get planning component
    sciurus17 = MoveItPy(node_name='joint_values')
    logger.info('MoveItPy instance created')

    # アーム制御用 planning component
    arm = sciurus17.get_planning_component('l_arm_group')
    planning_scene_monitor = sciurus17.get_planning_scene_monitor()

    # instantiate a RobotModel instance for creating goal states
    robot_model = sciurus17.get_robot_model()

    arm_plan_request_params = PlanRequestParameters(
        sciurus17,
        'ompl_rrtc_default',
    )

    # 動作速度の調整
    arm_plan_request_params.max_acceleration_scaling_factor = 0.5  # Set 0.0 ~ 1.0
    arm_plan_request_params.max_velocity_scaling_factor = 0.5  # Set 0.0 ~ 1.0

    # SRDFに定義されている'l_arm_init_pose'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='l_arm_init_pose')
    plan_and_execute(
        sciurus17,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

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

    # 現在角度をベースに、目標角度を作成する
    joint_values = []
    with planning_scene_monitor.read_only() as scene:
        robot_state = scene.current_state
        joint_values = robot_state.get_joint_group_positions('l_arm_group')

    # 各関節角度を初期姿勢から順番に15[deg]ずつ動かす
    for joint_index, joint_name in enumerate(joint_names):
        arm.set_start_state_to_current_state()

        joint_values[joint_index] += target_joint_diff_value
        logger.info(f'Move {joint_name} by 15[deg]')
        robot_state = RobotState(robot_model)
        robot_state.set_joint_group_positions('l_arm_group', joint_values)
        arm.set_goal_state(robot_state=robot_state)

        plan_and_execute(
            sciurus17,
            arm,
            logger,
            single_plan_parameters=arm_plan_request_params,
        )

    # SRDFに定義されている'l_arm_init_pose'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='l_arm_init_pose')
    plan_and_execute(
        sciurus17,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
