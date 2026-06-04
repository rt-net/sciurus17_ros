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


# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
# generic ros libraries
import rclpy
from rclpy.logging import get_logger

from sciurus17_examples_py.utils import plan_and_execute


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('moveit_py.gripper_control')

    # instantiate MoveItPy instance and get planning component
    sciurus17 = MoveItPy(node_name='gripper_control')
    logger.info('MoveItPy instance created')

    # 腕制御用 planning component
    arm = sciurus17.get_planning_component('two_arm_group')
    # 左グリッパ制御用 planning component
    l_gripper = sciurus17.get_planning_component('l_gripper_group')
    # 右グリッパ制御用 planning component
    r_gripper = sciurus17.get_planning_component('r_gripper_group')

    robot_model = sciurus17.get_robot_model()

    plan_request_params = PlanRequestParameters(
        sciurus17,
        'ompl_rrtc_default',
    )
    gripper_plan_request_params = PlanRequestParameters(
        sciurus17,
        'ompl_rrtc_default',
    )
    # 動作速度の調整
    plan_request_params.max_acceleration_scaling_factor = 0.1    # Set 0.0 ~ 1.0
    plan_request_params.max_velocity_scaling_factor = 0.1    # Set 0.0 ~ 1.0

    # グリッパの開閉角
    R_GRIPPER_CLOSE = math.radians(0.0)
    R_GRIPPER_OPEN = math.radians(40.0)
    L_GRIPPER_CLOSE = math.radians(0.0)
    L_GRIPPER_OPEN = math.radians(-40.0)

    # SRDFに定義されている'two_arm_init_pose'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='two_arm_init_pose')
    plan_and_execute(
        sciurus17,
        arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 右グリッパ開閉
    for _ in range(2):
        r_gripper.set_start_state_to_current_state()
        robot_state = RobotState(robot_model)
        robot_state.set_joint_group_positions('r_gripper_group', [R_GRIPPER_OPEN])
        r_gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            sciurus17,
            r_gripper,
            logger,
            single_plan_parameters=gripper_plan_request_params,
        )

        r_gripper.set_start_state_to_current_state()
        robot_state = RobotState(robot_model)
        robot_state.set_joint_group_positions('r_gripper_group', [R_GRIPPER_CLOSE])
        r_gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            sciurus17,
            r_gripper,
            logger,
            single_plan_parameters=gripper_plan_request_params,
        )

    # 左グリッパ開閉
    for _ in range(2):
        l_gripper.set_start_state_to_current_state()
        robot_state = RobotState(robot_model)
        robot_state.set_joint_group_positions('l_gripper_group', [L_GRIPPER_OPEN])
        l_gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            sciurus17,
            l_gripper,
            logger,
            single_plan_parameters=gripper_plan_request_params,
        )

        l_gripper.set_start_state_to_current_state()
        robot_state = RobotState(robot_model)
        robot_state.set_joint_group_positions('l_gripper_group', [L_GRIPPER_CLOSE])
        l_gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            sciurus17,
            l_gripper,
            logger,
            single_plan_parameters=gripper_plan_request_params,
        )

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
