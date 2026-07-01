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

from geometry_msgs.msg import PoseStamped

from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
from moveit_msgs.msg import Constraints, JointConstraint

import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sciurus17_examples_py.utils import plan_and_execute
from tf2_ros import TransformException, TransformListener, TransformStamped
from tf2_ros.buffer import Buffer


class ArmSide:
    LEFT = 'left'
    RIGHT = 'right'


class PickAndPlaceTf(Node):
    def __init__(self):
        super().__init__('pick_and_place_tf')
        self.logger = self.get_logger()

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_past = TransformStamped()

        # instantiate MoveItPy instance and get planning component
        self.sciurus17 = MoveItPy(node_name='moveit_py')
        self.logger.info('MoveItPy instance created')

        # 首制御用 planning component
        self.neck = self.sciurus17.get_planning_component('neck_group')
        # 左腕と腰制御用 planning component
        self.l_arm_waist = self.sciurus17.get_planning_component('l_arm_waist_group')
        # 左グリッパ制御用 planning component
        self.l_gripper = self.sciurus17.get_planning_component('l_gripper_group')
        # 右腕と腰制御用 planning component
        self.r_arm_waist = self.sciurus17.get_planning_component('r_arm_waist_group')
        # 右グリッパ制御用 planning component
        self.r_gripper = self.sciurus17.get_planning_component('r_gripper_group')

        # instantiate a RobotState instance using the current robot model
        self.robot_model = self.sciurus17.get_robot_model()

        self.arm_plan_request_params = PlanRequestParameters(
            self.sciurus17,
            'ompl_rrtc_default',
        )
        self.gripper_plan_request_params = PlanRequestParameters(
            self.sciurus17,
            'ompl_rrtc_default',
        )

        # 動作速度の調整
        self.arm_plan_request_params.max_acceleration_scaling_factor = 0.1  # Set 0.0 ~ 1.0
        self.arm_plan_request_params.max_velocity_scaling_factor = 0.1  # Set 0.0 ~ 1.0
        self.gripper_plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
        self.gripper_plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        # 可動範囲を制限する
        constraints = Constraints()
        constraints.name = 'arm_constraints'

        # 腰軸の可動範囲を制限する
        jointConstraint = JointConstraint()
        jointConstraint.joint_name = 'waist_yaw_joint'
        jointConstraint.position = 0.0
        jointConstraint.tolerance_above = math.radians(45)
        jointConstraint.tolerance_below = math.radians(45)
        jointConstraint.weight = 1.0
        constraints.joint_constraints.append(jointConstraint)

        self.l_arm_waist.set_path_constraints(path_constraints=constraints)
        self.r_arm_waist.set_path_constraints(path_constraints=constraints)

        # 姿勢を初期化
        self.init_body()

        # Call on_timer function every 0.5 second
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # target_0のtf位置姿勢を取得
        try:
            tf_msg = self.tf_buffer.lookup_transform('base_link', 'target_0', rclpy.time.Time())
        except TransformException as ex:
            self.logger.info(f'Could not transform base_link to target: {ex}')
            return

        now = self.get_clock().now()
        FILTERING_TIME = rclpy.duration.Duration(seconds=2)
        STOP_TIME_THRESHOLD = rclpy.duration.Duration(seconds=3)
        DISTANCE_THRESHOLD = 0.01
        TARGET_Z_MIN_LIMIT = 0.02
        TARGET_X_MIN_LIMIT = 0.13
        TARGET_X_MAX_LIMIT = 0.3
        # 経過時間と停止時間を計算(nsec)
        # 経過時間

        tf_elapsed_time = now - rclpy.time.Time.from_msg(tf_msg.header.stamp)
        # 停止時間
        tf_stop_time = now - rclpy.time.Time.from_msg(self.tf_past.header.stamp)

        # 掴む物体位置を制限する
        if tf_msg.transform.translation.z < TARGET_Z_MIN_LIMIT:
            return
        if (
            tf_msg.transform.translation.x < TARGET_X_MIN_LIMIT
            or tf_msg.transform.translation.x > TARGET_X_MAX_LIMIT
        ):
            return

        # 検出されてから2秒以上経過した物体は掴まない
        if tf_elapsed_time > FILTERING_TIME:
            return

        tf_diff = np.linalg.norm(
            [
                self.tf_past.transform.translation.x - tf_msg.transform.translation.x,
                self.tf_past.transform.translation.y - tf_msg.transform.translation.y,
                self.tf_past.transform.translation.z - tf_msg.transform.translation.z,
            ]
        )

        # 動いている物体は掴まない
        if tf_diff > DISTANCE_THRESHOLD:
            self.tf_past = tf_msg
            return

        # 物体が3秒以上停止している場合ピッキング動作開始
        if tf_stop_time < STOP_TIME_THRESHOLD:
            return

        self._picking(tf_msg.transform.translation)

    def _picking(self, target_position):
        GRIPPER_CLOSE = 0.0
        GRIPPER_OPEN = math.radians(50.0)
        GRIPPER_GRASP = math.radians(20.0)

        PLACE_POSITION_X = 0.35
        PLACE_POSITION_Y = 0.0
        PLACE_POSITION_Z = 0.05
        APPROACH_OFFSET_Z = 0.12
        GRASP_OFFSET_Z = 0.08

        # 物体位置に応じて左右の腕を切り替え
        if target_position.y > 0:
            current_arm = ArmSide.LEFT
        else:
            current_arm = ArmSide.RIGHT

        # 何かを掴んでいた時のためにハンドを開閉
        self._control_gripper(current_arm, GRIPPER_OPEN)
        self._control_gripper(current_arm, GRIPPER_CLOSE)

        # 掴む準備をする
        self._control_arm(
            current_arm,
            target_position.x,
            target_position.y,
            target_position.z + APPROACH_OFFSET_Z,
        )

        # ハンドを開く
        self._control_gripper(current_arm, GRIPPER_OPEN)

        # 掴みに行く
        self._control_arm(
            current_arm,
            target_position.x,
            target_position.y,
            target_position.z + GRASP_OFFSET_Z,
        )

        # ハンドを閉じる
        self._control_gripper(current_arm, GRIPPER_GRASP)

        # 持ち上げる
        self._control_arm(
            current_arm,
            target_position.x,
            target_position.y,
            target_position.z + APPROACH_OFFSET_Z,
        )

        # 移動する
        self._control_arm(
            current_arm,
            PLACE_POSITION_X,
            PLACE_POSITION_Y,
            PLACE_POSITION_Z + APPROACH_OFFSET_Z,
        )

        # 下ろす
        self._control_arm(
            current_arm,
            PLACE_POSITION_X,
            PLACE_POSITION_Y,
            PLACE_POSITION_Z + GRASP_OFFSET_Z,
        )

        # ハンドを開く
        self._control_gripper(current_arm, GRIPPER_OPEN)

        # 少しだけハンドを持ち上げる
        self._control_arm(
            current_arm,
            PLACE_POSITION_X,
            PLACE_POSITION_Y,
            PLACE_POSITION_Z + APPROACH_OFFSET_Z,
        )

        # 初期姿勢に戻る
        self.init_arm(current_arm)

        # ハンドを閉じる
        self._control_gripper(current_arm, GRIPPER_CLOSE)

    # グリッパ制御
    def _control_gripper(self, current_arm, angle):
        robot_state = RobotState(self.robot_model)

        if current_arm == ArmSide.LEFT:
            self.l_gripper.set_start_state_to_current_state()
            robot_state.set_joint_group_positions('l_gripper_group', [-angle])
            self.l_gripper.set_goal_state(robot_state=robot_state)
            plan_and_execute(
                self.sciurus17,
                self.l_gripper,
                self.logger,
                single_plan_parameters=self.gripper_plan_request_params,
            )
        if current_arm == ArmSide.RIGHT:
            self.r_gripper.set_start_state_to_current_state()
            robot_state.set_joint_group_positions('r_gripper_group', [angle])
            self.r_gripper.set_goal_state(robot_state=robot_state)
            plan_and_execute(
                self.sciurus17,
                self.r_gripper,
                self.logger,
                single_plan_parameters=self.gripper_plan_request_params,
            )

    # アーム制御
    def _control_arm(self, current_arm, x, y, z):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z

        if current_arm == ArmSide.LEFT:
            quat = Rotation.from_euler('xyz', [-90, 0, 0], degrees=True).as_quat()
            goal_pose.pose.orientation.x = quat[0]
            goal_pose.pose.orientation.y = quat[1]
            goal_pose.pose.orientation.z = quat[2]
            goal_pose.pose.orientation.w = quat[3]
            self.l_arm_waist.set_start_state_to_current_state()
            self.l_arm_waist.set_goal_state(pose_stamped_msg=goal_pose, pose_link='l_link7')
            result = plan_and_execute(
                self.sciurus17,
                self.l_arm_waist,
                self.logger,
                single_plan_parameters=self.arm_plan_request_params,
            )
            return result
        if current_arm == ArmSide.RIGHT:
            quat = Rotation.from_euler('xyz', [90, 0, 0], degrees=True).as_quat()
            goal_pose.pose.orientation.x = quat[0]
            goal_pose.pose.orientation.y = quat[1]
            goal_pose.pose.orientation.z = quat[2]
            goal_pose.pose.orientation.w = quat[3]
            self.r_arm_waist.set_start_state_to_current_state()
            self.r_arm_waist.set_goal_state(pose_stamped_msg=goal_pose, pose_link='r_link7')
            result = plan_and_execute(
                self.sciurus17,
                self.r_arm_waist,
                self.logger,
                single_plan_parameters=self.arm_plan_request_params,
            )
            return result
        return None

    def init_body(self):
        INITIAL_YAW_ANGLE = math.radians(0.0)
        INITIAL_PITCH_ANGLE = math.radians(-80.0)

        self.l_arm_waist.set_start_state_to_current_state()
        self.l_arm_waist.set_goal_state(configuration_name='l_arm_waist_init_pose')
        plan_and_execute(
            self.sciurus17,
            self.l_arm_waist,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )

        self.r_arm_waist.set_start_state_to_current_state()
        self.r_arm_waist.set_goal_state(configuration_name='r_arm_waist_init_pose')
        plan_and_execute(
            self.sciurus17,
            self.r_arm_waist,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )

        joint_values = [
            INITIAL_YAW_ANGLE,
            INITIAL_PITCH_ANGLE,
        ]
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('neck_group', joint_values)
        self.neck.set_start_state_to_current_state()
        self.neck.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.sciurus17,
            self.neck,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )

    def init_arm(self, current_arm):
        if current_arm == ArmSide.LEFT:
            self.l_arm_waist.set_start_state_to_current_state()
            self.l_arm_waist.set_goal_state(configuration_name='l_arm_waist_init_pose')
            plan_and_execute(
                self.sciurus17,
                self.l_arm_waist,
                self.logger,
                single_plan_parameters=self.arm_plan_request_params,
            )
        if current_arm == ArmSide.RIGHT:
            self.r_arm_waist.set_start_state_to_current_state()
            self.r_arm_waist.set_goal_state(configuration_name='r_arm_waist_init_pose')
            plan_and_execute(
                self.sciurus17,
                self.r_arm_waist,
                self.logger,
                single_plan_parameters=self.arm_plan_request_params,
            )


def main(args=None):
    rclpy.init(args=args)

    pick_and_place_tf_node = PickAndPlaceTf()

    executor = MultiThreadedExecutor()
    executor.add_node(pick_and_place_tf_node)
    executor.spin()

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    pick_and_place_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
