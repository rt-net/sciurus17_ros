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
import copy
import math

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
from moveit_msgs.msg import Constraints, JointConstraint
import rclpy
from rclpy.logging import get_logger
from sciurus17_examples_py.utils import plan_and_execute


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('moveit_py.pick_and_place_right_arm_waist')

    # instantiate MoveItPy instance and get planning component
    sciurus17 = MoveItPy(node_name='pick_and_place_right_arm_waist')
    logger.info('MoveItPy instance created')

    # 右腕と腰制御用 planning component
    r_arm_waist = sciurus17.get_planning_component('r_arm_waist_group')
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
    GRIPPER_CLOSE = math.radians(0.0)
    GRIPPER_OPEN = math.radians(40.0)
    GRIPPER_GRASP = math.radians(20.0)
    # 物体を持ち上げる高さ
    LIFTING_HEIFHT = 0.25
    # 物体を掴む位置
    GRASP_POSE = Pose(position=Point(x=0.25, y=0.0, z=0.12),
                      orientation=Quaternion(x=0.707, y=0.0, z=0.0, w=0.707))    # downward
    PRE_AND_POST_GRASP_POSE = copy.deepcopy(GRASP_POSE)
    PRE_AND_POST_GRASP_POSE.position.z = LIFTING_HEIFHT
    # 物体を置く位置
    RELEASE_POSE = Pose(position=Point(x=0.35, y=0.0, z=0.12),
                        orientation=Quaternion(x=0.707, y=0.0, z=0.0, w=0.707))    # downward
    PRE_AND_POST_RELEASE_POSE = copy.deepcopy(RELEASE_POSE)
    PRE_AND_POST_RELEASE_POSE.position.z = LIFTING_HEIFHT

    # SRDFに定義されている'r_arm_waist_init_pose'の姿勢にする
    r_arm_waist.set_start_state_to_current_state()
    r_arm_waist.set_goal_state(configuration_name='r_arm_waist_init_pose')
    plan_and_execute(
        sciurus17,
        r_arm_waist,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 何かを掴んでいた時のためにハンドを開く
    r_gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('r_gripper_group', [GRIPPER_OPEN])
    r_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        r_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    constraints = Constraints()
    joint_constraint = JointConstraint()
    joint_constraint.joint_name = 'waist_yaw_joint'
    joint_constraint.position = 0.0
    joint_constraint.tolerance_above = math.radians(45.0)
    joint_constraint.tolerance_below = math.radians(45.0)
    joint_constraint.weight = 1.0
    constraints.joint_constraints.append(joint_constraint)

    # 物体の上に腕を伸ばす
    r_arm_waist.set_path_constraints(path_constraints=constraints)
    r_arm_waist.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'base_link'
    goal_pose.pose = PRE_AND_POST_GRASP_POSE
    r_arm_waist.set_goal_state(pose_stamped_msg=goal_pose, pose_link='r_link7')
    plan_and_execute(
        sciurus17,
        r_arm_waist,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 掴みに行く
    r_arm_waist.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'base_link'
    goal_pose.pose = GRASP_POSE
    r_arm_waist.set_goal_state(pose_stamped_msg=goal_pose, pose_link='r_link7')
    plan_and_execute(
        sciurus17,
        r_arm_waist,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # ハンドを閉じる
    r_gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('r_gripper_group', [GRIPPER_GRASP])
    r_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        r_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # 持ち上げる
    r_arm_waist.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'base_link'
    goal_pose.pose = PRE_AND_POST_GRASP_POSE
    r_arm_waist.set_goal_state(pose_stamped_msg=goal_pose, pose_link='r_link7')
    plan_and_execute(
        sciurus17,
        r_arm_waist,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 移動する
    r_arm_waist.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'base_link'
    goal_pose.pose = PRE_AND_POST_RELEASE_POSE
    r_arm_waist.set_goal_state(pose_stamped_msg=goal_pose, pose_link='r_link7')
    plan_and_execute(
        sciurus17,
        r_arm_waist,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 下ろす
    r_arm_waist.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'base_link'
    goal_pose.pose = RELEASE_POSE
    r_arm_waist.set_goal_state(pose_stamped_msg=goal_pose, pose_link='r_link7')
    plan_and_execute(
        sciurus17,
        r_arm_waist,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # ハンドを開く
    r_gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('r_gripper_group', [GRIPPER_OPEN])
    r_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        r_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # ハンドを持ち上げる
    r_arm_waist.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'base_link'
    goal_pose.pose = PRE_AND_POST_RELEASE_POSE
    r_arm_waist.set_goal_state(pose_stamped_msg=goal_pose, pose_link='r_link7')
    plan_and_execute(
        sciurus17,
        r_arm_waist,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # SRDFに定義されている'r_arm_waist_init_pose'の姿勢にする
    r_arm_waist.set_start_state_to_current_state()
    r_arm_waist.set_goal_state(configuration_name='r_arm_waist_init_pose')
    plan_and_execute(
        sciurus17,
        r_arm_waist,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # ハンドを閉じる
    r_gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('r_gripper_group', [GRIPPER_CLOSE])
    r_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        r_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
