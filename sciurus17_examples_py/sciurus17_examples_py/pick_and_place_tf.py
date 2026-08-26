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
from moveit.planning import MoveItPy
from moveit.planning import PlanRequestParameters
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import TransformException
from tf2_ros import TransformListener
from tf2_ros import TransformStamped
from tf2_ros.buffer import Buffer

from sciurus17_examples_py.utils import plan_and_execute  # noqa: I100


# 左右の腕を識別するための定数クラス
class ArmSide:
    LEFT = 'left'
    RIGHT = 'right'


class PickAndPlaceTf(Node):
    """TFで検出した対象物を掴んで指定位置に置くノード."""

    # グリッパの開閉角度[rad]
    GRIPPER_CLOSE = 0.0
    GRIPPER_OPEN = math.radians(50.0)
    GRIPPER_GRASP = math.radians(20.0)

    # 置く位置（プレース位置）のXYZ[m]
    PLACE_X = 0.35
    PLACE_Y = 0.0
    PLACE_Z = 0.05

    # ピック・プレース動作時のZ軸オフセット[m]
    APPROACH_OFFSET_Z = 0.12  # アプローチ時の高さオフセット
    GRASP_OFFSET_Z = 0.08  # 把持時の高さオフセット

    def __init__(self):
        super().__init__('pick_and_place_tf')
        self.logger = self.get_logger()

        # TFリスナーの初期化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_past = TransformStamped()

        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.sciurus17 = MoveItPy(node_name='moveit_py')
        self.logger.info('MoveItPy instance created')

        # 首、左右アーム・グリッパ制御用 planning component
        self.neck = self.sciurus17.get_planning_component('neck_group')
        self.l_arm_waist = self.sciurus17.get_planning_component(
            'l_arm_waist_group'
        )
        self.l_gripper = self.sciurus17.get_planning_component(
            'l_gripper_group'
        )
        self.r_arm_waist = self.sciurus17.get_planning_component(
            'r_arm_waist_group'
        )
        self.r_gripper = self.sciurus17.get_planning_component(
            'r_gripper_group'
        )

        # ロボットモデルの取得（ジョイント目標値の設定に使用）
        self.robot_model = self.sciurus17.get_robot_model()

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.arm_plan_params = PlanRequestParameters(
            self.sciurus17,
            'ompl_rrtc_default',
        )
        self.gripper_plan_params = PlanRequestParameters(
            self.sciurus17,
            'ompl_rrtc_default',
        )

        # 0.0〜1.0の範囲で設定
        self.arm_plan_params.max_acceleration_scaling_factor = 0.1

        self.arm_plan_params.max_velocity_scaling_factor = 0.1  # 0.0〜1.0の範囲で設定
        # 0.0〜1.0の範囲で設定
        self.gripper_plan_params.max_acceleration_scaling_factor = 1.0

        # 0.0〜1.0の範囲で設定
        self.gripper_plan_params.max_velocity_scaling_factor = 1.0

        # 腰軸の可動範囲を制限する
        self.set_constraints()

        # 初期姿勢に移動
        self.init_body()

        # 0.5秒ごとにon_timerを呼び出すタイマーを作成
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # target_0のTF位置姿勢を取得
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'base_link', 'target_0', rclpy.time.Time()
            )
        except TransformException as ex:
            self.logger.info(f'Could not transform base_link to target: {ex}')
            return

        # TFフィルタリング用の定数
        now = self.get_clock().now()
        FILTERING_TIME = rclpy.duration.Duration(seconds=2)
        STOP_TIME_THRESHOLD = rclpy.duration.Duration(seconds=3)
        DISTANCE_THRESHOLD = 0.01
        TARGET_Z_MIN_LIMIT = 0.02
        TARGET_X_MIN_LIMIT = 0.13
        TARGET_X_MAX_LIMIT = 0.3

        # TF受信からの経過時間と物体の停止時間を計算
        tf_elapsed_time = now - rclpy.time.Time.from_msg(tf_msg.header.stamp)
        tf_stop_time = now - rclpy.time.Time.from_msg(
            self.tf_past.header.stamp
        )

        # 把持対象の位置が可動範囲外の場合は処理しない
        if tf_msg.transform.translation.z < TARGET_Z_MIN_LIMIT:
            return
        if (
            tf_msg.transform.translation.x < TARGET_X_MIN_LIMIT
            or tf_msg.transform.translation.x > TARGET_X_MAX_LIMIT
        ):
            return

        # 現在時刻から2秒以内に受け取ったTFを使用
        if tf_elapsed_time > FILTERING_TIME:
            return

        # 前回のTF位置との差分を計算
        tf_diff = np.linalg.norm(
            [
                self.tf_past.transform.translation.x
                - tf_msg.transform.translation.x,
                self.tf_past.transform.translation.y
                - tf_msg.transform.translation.y,
                self.tf_past.transform.translation.z
                - tf_msg.transform.translation.z,
            ]
        )

        # 把持対象の位置が停止していることを判定
        if tf_diff > DISTANCE_THRESHOLD:
            self.tf_past = tf_msg
            return

        # 把持対象が3秒以上停止している場合ピッキング動作開始
        if tf_stop_time < STOP_TIME_THRESHOLD:
            return

        self.picking(tf_msg.transform.translation)

    def picking(self, target_position):
        # 物体位置に応じて左右の腕を切り替え
        if target_position.y > 0:
            current_arm = ArmSide.LEFT
        else:
            current_arm = ArmSide.RIGHT

        # 何かを掴んでいた時のためにハンドを開閉
        self.move_gripper_angle(current_arm, self.GRIPPER_OPEN)
        self.move_gripper_angle(current_arm, self.GRIPPER_CLOSE)

        # ピック動作（掴みに行く）
        self.control_arm(
            current_arm,
            target_position.x,
            target_position.y,
            target_position.z + self.APPROACH_OFFSET_Z,
        )
        self.move_gripper_angle(current_arm, self.GRIPPER_OPEN)
        self.control_arm(
            current_arm,
            target_position.x,
            target_position.y,
            target_position.z + self.GRASP_OFFSET_Z,
        )
        self.move_gripper_angle(current_arm, self.GRIPPER_GRASP)
        self.control_arm(
            current_arm,
            target_position.x,
            target_position.y,
            target_position.z + self.APPROACH_OFFSET_Z,
        )

        # プレース動作（移動して置く）
        self.control_arm(
            current_arm,
            self.PLACE_X,
            self.PLACE_Y,
            self.PLACE_Z + self.APPROACH_OFFSET_Z,
        )
        self.control_arm(
            current_arm,
            self.PLACE_X,
            self.PLACE_Y,
            self.PLACE_Z + self.GRASP_OFFSET_Z,
        )
        self.move_gripper_angle(current_arm, self.GRIPPER_OPEN)
        self.control_arm(
            current_arm,
            self.PLACE_X,
            self.PLACE_Y,
            self.PLACE_Z + self.APPROACH_OFFSET_Z,
        )

        # 初期姿勢に戻る
        self.init_arm(current_arm)
        self.move_gripper_angle(current_arm, self.GRIPPER_CLOSE)

    def move_gripper_angle(self, current_arm, angle):
        # グリッパを角度[rad]を指定して開閉する
        robot_state = RobotState(self.robot_model)

        if current_arm == ArmSide.LEFT:
            self.l_gripper.set_start_state_to_current_state()
            robot_state.set_joint_group_positions('l_gripper_group', [-angle])
            self.l_gripper.set_goal_state(robot_state=robot_state)
            plan_and_execute(
                self.sciurus17,
                self.l_gripper,
                self.logger,
                single_plan_parameters=self.gripper_plan_params,
            )
        if current_arm == ArmSide.RIGHT:
            self.r_gripper.set_start_state_to_current_state()
            robot_state.set_joint_group_positions('r_gripper_group', [angle])
            self.r_gripper.set_goal_state(robot_state=robot_state)
            plan_and_execute(
                self.sciurus17,
                self.r_gripper,
                self.logger,
                single_plan_parameters=self.gripper_plan_params,
            )

    def control_arm(self, current_arm, x, y, z):
        # アームを目標位置（x, y, z [m]）に動かす
        # 姿勢は左右アームで固定（左腕: roll=-90deg, 右腕: roll=+90deg）
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z

        if current_arm == ArmSide.LEFT:
            quat = Rotation.from_euler(
                'xyz', [-90, 0, 0], degrees=True
            ).as_quat()
            goal_pose.pose.orientation.x = quat[0]
            goal_pose.pose.orientation.y = quat[1]
            goal_pose.pose.orientation.z = quat[2]
            goal_pose.pose.orientation.w = quat[3]
            self.l_arm_waist.set_start_state_to_current_state()
            self.l_arm_waist.set_goal_state(
                pose_stamped_msg=goal_pose, pose_link='l_link7'
            )
            result = plan_and_execute(
                self.sciurus17,
                self.l_arm_waist,
                self.logger,
                single_plan_parameters=self.arm_plan_params,
            )
            return result
        if current_arm == ArmSide.RIGHT:
            quat = Rotation.from_euler(
                'xyz', [90, 0, 0], degrees=True
            ).as_quat()
            goal_pose.pose.orientation.x = quat[0]
            goal_pose.pose.orientation.y = quat[1]
            goal_pose.pose.orientation.z = quat[2]
            goal_pose.pose.orientation.w = quat[3]
            self.r_arm_waist.set_start_state_to_current_state()
            self.r_arm_waist.set_goal_state(
                pose_stamped_msg=goal_pose, pose_link='r_link7'
            )
            result = plan_and_execute(
                self.sciurus17,
                self.r_arm_waist,
                self.logger,
                single_plan_parameters=self.arm_plan_params,
            )
            return result
        return None

    def init_body(self):
        # カメラで把持対象を撮影するための待機姿勢に移動する
        INITIAL_YAW_ANGLE = math.radians(0.0)
        INITIAL_PITCH_ANGLE = math.radians(-80.0)

        # 左腕を初期姿勢に移動
        self.l_arm_waist.set_start_state_to_current_state()
        self.l_arm_waist.set_goal_state(
            configuration_name='l_arm_waist_init_pose'
        )
        plan_and_execute(
            self.sciurus17,
            self.l_arm_waist,
            self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

        # 右腕を初期姿勢に移動
        self.r_arm_waist.set_start_state_to_current_state()
        self.r_arm_waist.set_goal_state(
            configuration_name='r_arm_waist_init_pose'
        )
        plan_and_execute(
            self.sciurus17,
            self.r_arm_waist,
            self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

        # 首を下向きに設定
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
            single_plan_parameters=self.arm_plan_params,
        )

    def init_arm(self, current_arm):
        # SRDFに定義された初期姿勢にアームを動かす
        if current_arm == ArmSide.LEFT:
            self.l_arm_waist.set_start_state_to_current_state()
            self.l_arm_waist.set_goal_state(
                configuration_name='l_arm_waist_init_pose'
            )
            plan_and_execute(
                self.sciurus17,
                self.l_arm_waist,
                self.logger,
                single_plan_parameters=self.arm_plan_params,
            )
        if current_arm == ArmSide.RIGHT:
            self.r_arm_waist.set_start_state_to_current_state()
            self.r_arm_waist.set_goal_state(
                configuration_name='r_arm_waist_init_pose'
            )
            plan_and_execute(
                self.sciurus17,
                self.r_arm_waist,
                self.logger,
                single_plan_parameters=self.arm_plan_params,
            )

    def set_constraints(self):
        # 腰軸の可動範囲を制限する
        constraints = Constraints()
        constraints.name = 'arm_constraints'

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'waist_yaw_joint'
        joint_constraint.position = 0.0
        joint_constraint.tolerance_above = math.radians(45)
        joint_constraint.tolerance_below = math.radians(45)
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)

        self.l_arm_waist.set_path_constraints(path_constraints=constraints)
        self.r_arm_waist.set_path_constraints(path_constraints=constraints)

    def clear_constraints(self):
        # 設定された関節可動制限をクリアする
        self.l_arm_waist.clear_path_constraints()
        self.r_arm_waist.clear_path_constraints()


def main(args=None):
    rclpy.init(args=args)

    pick_and_place_tf_node = PickAndPlaceTf()

    executor = MultiThreadedExecutor()
    executor.add_node(pick_and_place_tf_node)
    executor.spin()

    pick_and_place_tf_node.clear_constraints()

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    pick_and_place_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
