// Copyright 2024 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Reference:
// https://github.com/ros-planning/moveit2_tutorials/blob
// /a547cf49ff7d1fe16a93dfe020c6027bcb035b51/doc/move_group_interface
// /src/move_group_interface_tutorial.cpp
// https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html

#include <angles/angles.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/convert.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "pose_presets.hpp"

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlaceTf : public rclcpp::Node
{
public:
  // 左腕・右腕を区別する列挙型
  enum class ArmSide
  {
    LEFT,
    RIGHT
  };

  // グリッパの開閉角度[rad]
  inline static const double GRIPPER_CLOSE = 0.0;
  inline static const double GRIPPER_OPEN = angles::from_degrees(50.0);
  inline static const double GRIPPER_GRASP = angles::from_degrees(20.0);

  // 置く位置（プレース位置）のXYZ[m]
  inline static const double PLACE_X = 0.35;
  inline static const double PLACE_Y = 0.0;
  inline static const double PLACE_Z = 0.05;

  // 物体位置からの相対オフセット[m]（ピック動作時に使用）
  inline static const double APPROACH_OFFSET_Z = 0.12;
  inline static const double GRASP_OFFSET_Z = 0.08;

  PickAndPlaceTf(
    rclcpp::Node::SharedPtr move_group_neck_node, rclcpp::Node::SharedPtr move_group_l_arm_node,
    rclcpp::Node::SharedPtr move_group_l_gripper_node,
    rclcpp::Node::SharedPtr move_group_r_arm_node,
    rclcpp::Node::SharedPtr move_group_r_gripper_node)
  : Node("pick_and_place_tf_node")
  {
    using namespace std::placeholders;

    // 首のMoveGroupInterfaceを初期化
    move_group_neck_ = std::make_shared<MoveGroupInterface>(move_group_neck_node, "neck_group");
    move_group_neck_->setMaxVelocityScalingFactor(0.1);
    move_group_neck_->setMaxAccelerationScalingFactor(0.1);

    // 左腕のMoveGroupInterfaceを初期化
    move_group_l_arm_ =
      std::make_shared<MoveGroupInterface>(move_group_l_arm_node, "l_arm_waist_group");
    move_group_l_arm_->setMaxVelocityScalingFactor(0.1);
    move_group_l_arm_->setMaxAccelerationScalingFactor(0.1);

    // 左グリッパのMoveGroupInterfaceを初期化
    move_group_l_gripper_ =
      std::make_shared<MoveGroupInterface>(move_group_l_gripper_node, "l_gripper_group");

    // 右腕のMoveGroupInterfaceを初期化
    move_group_r_arm_ =
      std::make_shared<MoveGroupInterface>(move_group_r_arm_node, "r_arm_waist_group");
    move_group_r_arm_->setMaxVelocityScalingFactor(0.1);
    move_group_r_arm_->setMaxAccelerationScalingFactor(0.1);

    // 右グリッパのMoveGroupInterfaceを初期化
    move_group_r_gripper_ =
      std::make_shared<MoveGroupInterface>(move_group_r_gripper_node, "r_gripper_group");

    // SRDFに定義されている姿勢に体全体を初期化
    init_body();

    // 腰軸と首軸の可動範囲を制限
    set_constraints();

    // TFリスナーを初期化（物体位置を取得するため）
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), 2s);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // タイマーを起動（500msごとにon_timerを呼び出し）
    timer_ = this->create_wall_timer(500ms, std::bind(&PickAndPlaceTf::on_timer, this));
  }

  // グリッパを角度[rad]を指定して開閉する
  void move_gripper_angle(const ArmSide current_arm, const double angle)
  {
    auto joint_values = move_group_l_gripper_->getCurrentJointValues();

    if (current_arm == ArmSide::LEFT) {
      joint_values[0] = -angle;
      move_group_l_gripper_->setJointValueTarget(joint_values);
      move_group_l_gripper_->move();
    }
    if (current_arm == ArmSide::RIGHT) {
      joint_values[0] = angle;
      move_group_r_gripper_->setJointValueTarget(joint_values);
      move_group_r_gripper_->move();
    }
  }

  // アームを目標位置（x, y, z [m]）に動かす（姿勢は下向き固定）
  void control_arm(const ArmSide current_arm, const double x, const double y, const double z)
  {
    if (current_arm == ArmSide::LEFT) {
      move_group_l_arm_->setPoseTarget(pose_presets::left_arm_downward(x, y, z));
      move_group_l_arm_->move();
    }
    if (current_arm == ArmSide::RIGHT) {
      move_group_r_arm_->setPoseTarget(pose_presets::right_arm_downward(x, y, z));
      move_group_r_arm_->move();
    }
  }

  // SRDFに定義された姿勢名でアームを動かす
  void move_arm_to_named_pose(const ArmSide current_arm, const std::string & name)
  {
    if (current_arm == ArmSide::LEFT) {
      move_group_l_arm_->setNamedTarget(name);
      move_group_l_arm_->move();
    }
    if (current_arm == ArmSide::RIGHT) {
      move_group_r_arm_->setNamedTarget(name);
      move_group_r_arm_->move();
    }
  }

  // アームの関節の一部に可動制限を設定する
  void set_constraints()
  {
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "arm_constraints";

    // 腰軸の可動範囲を±45度に制限する
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = "waist_yaw_joint";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(45);
    joint_constraint.tolerance_below = angles::from_degrees(45);
    joint_constraint.weight = 1.0;
    constraints.joint_constraints.push_back(joint_constraint);

    move_group_l_arm_->setPathConstraints(constraints);
    move_group_r_arm_->setPathConstraints(constraints);
  }

  // 設定された関節可動制限をクリアする
  void clear_constraints()
  {
    move_group_l_arm_->clearPathConstraints();
    move_group_r_arm_->clearPathConstraints();
  }

private:
  void on_timer()
  {
    // target_0のTF（Transform）を取得して、物体の位置を調べる
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      // base_linkからtarget_0へのTFを取得
      tf_msg = tf_buffer_->lookupTransform("base_link", "target_0", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform base_link to target: %s", ex.what());
      return;
    }

    // 現在時刻を取得
    rclcpp::Time now = this->get_clock()->now();

    // 物体検出のフィルタリング時間（この時間以内のTFのみ有効とする）
    const auto FILTERING_TIME = rclcpp::Duration(2s);
    // 物体が停止していると判定する時間の閾値
    const auto STOP_TIME_THRESHOLD = rclcpp::Duration(3s);
    // 物体が動いているかを判定する距離の閾値[m]
    const double DISTANCE_THRESHOLD = 0.01;
    // 物体のZ座標の最小値[m]（これより低い物体は無視する）
    const double TARGET_Z_MIN_LIMIT = 0.02;
    // 物体のX座標の最小値・最大値[m]（範囲外の物体は無視する）
    const double TARGET_X_MIN_LIMIT = 0.13;
    const double TARGET_X_MAX_LIMIT = 0.3;

    tf2::Stamped<tf2::Transform> tf_current;
    tf2::convert(tf_msg, tf_current);

    // TFのスタンプから経過時間を計算
    const auto tf_elapsed_time = now - rclcpp::Time(tf_msg.header.stamp, RCL_ROS_TIME);
    // 前回のTFから停止している時間を計算
    const auto tf_stop_time =
      now - rclcpp::Time(tf_past_.stamp_.time_since_epoch().count(), RCL_ROS_TIME);

    // 掴む物体の位置がロボットの可動範囲内かをチェック
    if (tf_current.getOrigin().z() < TARGET_Z_MIN_LIMIT) {
      return;
    }
    if (
      tf_current.getOrigin().x() < TARGET_X_MIN_LIMIT ||
      tf_current.getOrigin().x() > TARGET_X_MAX_LIMIT)
    {
      return;
    }

    // 検出されてから2秒以上経過した物体は無視（古い情報のため）
    if (tf_elapsed_time > FILTERING_TIME) {
      return;
    }

    // 前回の位置と現在の位置の距離を計算
    double tf_diff = (tf_past_.getOrigin() - tf_current.getOrigin()).length();

    // 動いている物体は掴まない（距離が閾値より大きい場合）
    if (tf_diff > DISTANCE_THRESHOLD) {
      tf_past_ = tf_current;
      return;
    }

    // 物体が3秒以上停止している場合、ピッキング動作を開始
    if (tf_stop_time < STOP_TIME_THRESHOLD) {
      return;
    }

    picking(tf_current.getOrigin());
  }

  void init_body()
  {
    // 首の初期角度を設定
    const double INITIAL_YAW_ANGLE = angles::from_degrees(0.0);
    const double INITIAL_PITCH_ANGLE = angles::from_degrees(-80.0);

    // 左腕・右腕をSRDFに定義された初期姿勢に移動
    move_group_l_arm_->setNamedTarget("l_arm_waist_init_pose");
    move_group_l_arm_->move();
    move_group_r_arm_->setNamedTarget("r_arm_waist_init_pose");
    move_group_r_arm_->move();

    // 首をやや下向きに設定（物体を見やすくするため）
    std::vector<double> joint_values;
    joint_values.push_back(INITIAL_YAW_ANGLE);
    joint_values.push_back(INITIAL_PITCH_ANGLE);
    move_group_neck_->setJointValueTarget(joint_values);
    move_group_neck_->move();
  }

  void picking(tf2::Vector3 target_position)
  {
    // 物体のY座標に応じて左右の腕を選択
    // Y座標が正の値なら左腕、負の値なら右腕を使用
    ArmSide current_arm;
    if (target_position.y() > 0) {
      current_arm = ArmSide::LEFT;
    } else {
      current_arm = ArmSide::RIGHT;
    }

    // 何かを掴んでいた時のためにグリッパを開閉してリセット
    move_gripper_angle(current_arm, GRIPPER_OPEN);
    move_gripper_angle(current_arm, GRIPPER_CLOSE);

    // ピック動作（物体を掴む）

    // 物体の上方に移動（アプローチ位置）
    control_arm(
      current_arm, target_position.x(), target_position.y(),
      target_position.z() + APPROACH_OFFSET_Z);

    // グリッパを開く
    move_gripper_angle(current_arm, GRIPPER_OPEN);

    // 掴む高さまで下降
    control_arm(
      current_arm, target_position.x(), target_position.y(), target_position.z() + GRASP_OFFSET_Z);

    // グリッパを閉じて物体を掴む
    move_gripper_angle(current_arm, GRIPPER_GRASP);

    // 物体を持ち上げる
    control_arm(
      current_arm, target_position.x(), target_position.y(),
      target_position.z() + APPROACH_OFFSET_Z);

    // プレース動作（物体を置く）

    // プレース位置の上方に移動
    control_arm(current_arm, PLACE_X, PLACE_Y, PLACE_Z + APPROACH_OFFSET_Z);

    // プレース位置まで下降
    control_arm(current_arm, PLACE_X, PLACE_Y, PLACE_Z + GRASP_OFFSET_Z);

    // グリッパを開いて物体を離す
    move_gripper_angle(current_arm, GRIPPER_OPEN);

    // グリッパを少し持ち上げる
    control_arm(current_arm, PLACE_X, PLACE_Y, PLACE_Z + APPROACH_OFFSET_Z);

    // 待機姿勢（初期姿勢）に戻る
    move_arm_to_named_pose(
      current_arm,
      current_arm == ArmSide::LEFT ? "l_arm_waist_init_pose" : "r_arm_waist_init_pose");

    // グリッパを閉じる
    move_gripper_angle(current_arm, GRIPPER_CLOSE);
  }

  // MoveGroupInterfaceのインスタンス（首、左右の腕とグリッパ）
  std::shared_ptr<MoveGroupInterface> move_group_neck_;
  std::shared_ptr<MoveGroupInterface> move_group_l_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_l_gripper_;
  std::shared_ptr<MoveGroupInterface> move_group_r_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_r_gripper_;

  // TFリスナー関連（物体位置を取得するため）
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  // タイマー（定期的に物体位置をチェックするため）
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  // 前回取得した物体のTF（位置が停止しているかを判定するため）
  tf2::Stamped<tf2::Transform> tf_past_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // 各MoveGroupInterface用のノードを作成
  auto move_group_neck_node = rclcpp::Node::make_shared("move_group_neck_node", node_options);
  auto move_group_l_arm_node = rclcpp::Node::make_shared("move_group_l_arm_node", node_options);
  auto move_group_l_gripper_node =
    rclcpp::Node::make_shared("move_group_l_gripper_node", node_options);
  auto move_group_r_arm_node = rclcpp::Node::make_shared("move_group_r_arm_node", node_options);
  auto move_group_r_gripper_node =
    rclcpp::Node::make_shared("move_group_r_gripper_node", node_options);

  // タイマーとTFリスナーを持つため、MultiThreadedExecutorを使用する
  rclcpp::executors::MultiThreadedExecutor exec;
  auto pick_and_place_tf_node = std::make_shared<PickAndPlaceTf>(
    move_group_neck_node, move_group_l_arm_node, move_group_l_gripper_node, move_group_r_arm_node,
    move_group_r_gripper_node);

  // 各ノードをExecutorに追加
  exec.add_node(pick_and_place_tf_node);
  exec.add_node(move_group_neck_node);
  exec.add_node(move_group_l_arm_node);
  exec.add_node(move_group_l_gripper_node);
  exec.add_node(move_group_r_arm_node);
  exec.add_node(move_group_r_gripper_node);
  exec.spin();

  // 終了時に可動制限をクリア
  pick_and_place_tf_node->clear_constraints();
  rclcpp::shutdown();
  return 0;
}
