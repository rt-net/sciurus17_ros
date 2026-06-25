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

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "pose_presets.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlaceTf : public rclcpp::Node
{
public:
  PickAndPlaceTf(
    rclcpp::Node::SharedPtr move_group_neck_node,
    rclcpp::Node::SharedPtr move_group_l_arm_node,
    rclcpp::Node::SharedPtr move_group_l_gripper_node,
    rclcpp::Node::SharedPtr move_group_r_arm_node,
    rclcpp::Node::SharedPtr move_group_r_gripper_node)
  : Node("pick_and_place_tf_node")
  {
    using namespace std::placeholders;
    move_group_neck_ =
      std::make_shared<MoveGroupInterface>(move_group_neck_node, "neck_group");
    move_group_neck_->setMaxVelocityScalingFactor(0.1);
    move_group_neck_->setMaxAccelerationScalingFactor(0.1);

    move_group_l_arm_ =
      std::make_shared<MoveGroupInterface>(move_group_l_arm_node, "l_arm_waist_group");
    move_group_l_arm_->setMaxVelocityScalingFactor(0.1);
    move_group_l_arm_->setMaxAccelerationScalingFactor(0.1);

    move_group_l_gripper_ =
      std::make_shared<MoveGroupInterface>(move_group_l_gripper_node, "l_gripper_group");

    move_group_r_arm_ =
      std::make_shared<MoveGroupInterface>(move_group_r_arm_node, "r_arm_waist_group");
    move_group_r_arm_->setMaxVelocityScalingFactor(0.1);
    move_group_r_arm_->setMaxAccelerationScalingFactor(0.1);

    move_group_r_gripper_ =
      std::make_shared<MoveGroupInterface>(move_group_r_gripper_node, "r_gripper_group");

    // 姿勢を初期化
    init_body();

    // 可動範囲を制限する
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "arm_constraints";

    // 腰軸の可動範囲を制限する
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = "waist_yaw_joint";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(45);
    joint_constraint.tolerance_below = angles::from_degrees(45);
    joint_constraint.weight = 1.0;
    constraints.joint_constraints.push_back(joint_constraint);

    move_group_l_arm_->setPathConstraints(constraints);
    move_group_r_arm_->setPathConstraints(constraints);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock(), 2s);
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&PickAndPlaceTf::on_timer, this));
  }

private:
  enum class ArmSide
  {
    LEFT,
    RIGHT
  };

  void on_timer()
  {
    // target_0のtf位置姿勢を取得
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_->lookupTransform(
        "base_link", "target_0",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform base_link to target: %s",
        ex.what());
      return;
    }

    rclcpp::Time now = this->get_clock()->now();
    const auto FILTERING_TIME = rclcpp::Duration(2s);
    const auto STOP_TIME_THRESHOLD = rclcpp::Duration(3s);
    const double DISTANCE_THRESHOLD = 0.01;
    const double TARGET_Z_MIN_LIMIT = 0.02;
    const double TARGET_X_MIN_LIMIT = 0.13;
    const double TARGET_X_MAX_LIMIT = 0.3;

    tf2::Stamped<tf2::Transform> tf_current;
    tf2::convert(tf_msg, tf_current);

    const auto tf_elapsed_time = now - rclcpp::Time(tf_msg.header.stamp, RCL_ROS_TIME);
    const auto tf_stop_time =
      now - rclcpp::Time(tf_past_.stamp_.time_since_epoch().count(), RCL_ROS_TIME);

    // 掴む物体位置を制限する
    if (tf_current.getOrigin().z() < TARGET_Z_MIN_LIMIT) {
      return;
    }
    if (tf_current.getOrigin().x() < TARGET_X_MIN_LIMIT ||
      tf_current.getOrigin().x() > TARGET_X_MAX_LIMIT)
    {
      return;
    }

    // 検出されてから2秒以上経過した物体は掴まない
    if (tf_elapsed_time > FILTERING_TIME) {
      return;
    }

    // 動いている物体は掴まない
    double tf_diff = (tf_past_.getOrigin() - tf_current.getOrigin()).length();
    if (tf_diff > DISTANCE_THRESHOLD) {
      tf_past_ = tf_current;
      return;
    }

    // 物体が3秒以上停止している場合ピッキング動作開始
    if (tf_stop_time < STOP_TIME_THRESHOLD) {
      return;
    }

    picking(tf_current.getOrigin());
  }

  void init_body()
  {
    const double INITIAL_YAW_ANGLE = angles::from_degrees(0.0);
    const double INITIAL_PITCH_ANGLE = angles::from_degrees(-80.0);

    move_group_l_arm_->setNamedTarget("l_arm_waist_init_pose");
    move_group_l_arm_->move();
    move_group_r_arm_->setNamedTarget("r_arm_waist_init_pose");
    move_group_r_arm_->move();

    std::vector<double> joint_values;
    joint_values.push_back(INITIAL_YAW_ANGLE);
    joint_values.push_back(INITIAL_PITCH_ANGLE);
    move_group_neck_->setJointValueTarget(joint_values);
    move_group_neck_->move();
  }

  void picking(tf2::Vector3 target_position)
  {
    // グリッパ開閉角度
    constexpr double GRIPPER_CLOSE = 0.0;
    const double GRIPPER_OPEN = angles::from_degrees(50.0);
    const double GRIPPER_GRASP = angles::from_degrees(20.0);

    // 物体を置く位置
    constexpr double PLACE_POSITION_X = 0.35;
    constexpr double PLACE_POSITION_Y = 0.0;
    constexpr double PLACE_POSITION_Z = 0.05;

    // 物体位置のオフセット
    constexpr double APPROACH_OFFSET_Z = 0.12;
    constexpr double GRASP_OFFSET_Z = 0.08;

    // 物体位置に応じて左右の腕を切り替え
    ArmSide current_arm;
    if (target_position.y() > 0) {
      current_arm = ArmSide::LEFT;
    } else {
      current_arm = ArmSide::RIGHT;
    }

    // 何かを掴んでいた時のためにハンドを開閉
    control_gripper(current_arm, GRIPPER_OPEN);
    control_gripper(current_arm, GRIPPER_CLOSE);

    // 掴む準備をする
    control_arm(
      current_arm,
      target_position.x(), target_position.y(), target_position.z() + APPROACH_OFFSET_Z);

    // ハンドを開く
    control_gripper(current_arm, GRIPPER_OPEN);

    // 掴みに行く
    control_arm(
      current_arm,
      target_position.x(), target_position.y(), target_position.z() + GRASP_OFFSET_Z);

    // ハンドを閉じる
    control_gripper(current_arm, GRIPPER_GRASP);

    // 持ち上げる
    control_arm(
      current_arm,
      target_position.x(), target_position.y(), target_position.z() + APPROACH_OFFSET_Z);

    // 移動する
    control_arm(
      current_arm,
      PLACE_POSITION_X, PLACE_POSITION_Y, PLACE_POSITION_Z + APPROACH_OFFSET_Z);

    // 下ろす
    control_arm(
      current_arm,
      PLACE_POSITION_X, PLACE_POSITION_Y, PLACE_POSITION_Z + GRASP_OFFSET_Z);

    // ハンドを開く
    control_gripper(current_arm, GRIPPER_OPEN);

    // 少しだけハンドを持ち上げる
    control_arm(
      current_arm,
      PLACE_POSITION_X, PLACE_POSITION_Y, PLACE_POSITION_Z + APPROACH_OFFSET_Z);

    // 初期姿勢に戻る
    init_arm(current_arm);

    // ハンドを閉じる
    control_gripper(current_arm, GRIPPER_CLOSE);
  }

  // グリッパ制御
  void control_gripper(const ArmSide current_arm, const double angle)
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

  // アーム制御
  void control_arm(
    const ArmSide current_arm, const double x, const double y, const double z)
  {
    if (current_arm == ArmSide::LEFT) {
      move_group_l_arm_->setPoseTarget(
        pose_presets::left_arm_downward(x, y, z));
      move_group_l_arm_->move();
    }
    if (current_arm == ArmSide::RIGHT) {
      move_group_r_arm_->setPoseTarget(
        pose_presets::right_arm_downward(x, y, z));
      move_group_r_arm_->move();
    }
  }

  void init_arm(const ArmSide current_arm)
  {
    if (current_arm == ArmSide::LEFT) {
      move_group_l_arm_->setNamedTarget("l_arm_waist_init_pose");
      move_group_l_arm_->move();
    }
    if (current_arm == ArmSide::RIGHT) {
      move_group_r_arm_->setNamedTarget("r_arm_waist_init_pose");
      move_group_r_arm_->move();
    }
  }

  std::shared_ptr<MoveGroupInterface> move_group_neck_;
  std::shared_ptr<MoveGroupInterface> move_group_l_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_l_gripper_;
  std::shared_ptr<MoveGroupInterface> move_group_r_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_r_gripper_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  tf2::Stamped<tf2::Transform> tf_past_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_neck_node =
    rclcpp::Node::make_shared("move_group_neck_node", node_options);
  auto move_group_l_arm_node =
    rclcpp::Node::make_shared("move_group_l_arm_node", node_options);
  auto move_group_l_gripper_node =
    rclcpp::Node::make_shared("move_group_l_gripper_node", node_options);
  auto move_group_r_arm_node =
    rclcpp::Node::make_shared("move_group_r_arm_node", node_options);
  auto move_group_r_gripper_node =
    rclcpp::Node::make_shared("move_group_r_gripper_node", node_options);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto pick_and_place_tf_node = std::make_shared<PickAndPlaceTf>(
    move_group_neck_node,
    move_group_l_arm_node,
    move_group_l_gripper_node,
    move_group_r_arm_node,
    move_group_r_gripper_node);
  exec.add_node(pick_and_place_tf_node);
  exec.add_node(move_group_neck_node);
  exec.add_node(move_group_l_arm_node);
  exec.add_node(move_group_l_gripper_node);
  exec.add_node(move_group_r_arm_node);
  exec.add_node(move_group_r_gripper_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
