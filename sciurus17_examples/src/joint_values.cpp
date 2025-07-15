// Copyright 2025 RT Corporation
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

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("joint_values");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("joint_values", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_arm(move_group_node, "l_arm_group");
  // 駆動速度を調整する
  move_group_arm.setMaxVelocityScalingFactor(0.5);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(0.5);  // Set 0.0 ~ 1.0

  // SRDFに定義されている"l_arm_init_pose"の姿勢にする
  move_group_arm.setNamedTarget("l_arm_init_pose");
  move_group_arm.move();

  // 現在角度をベースに、目標角度を設定する
  auto joint_values = move_group_arm.getCurrentJointValues();
  // 各ジョイントの角度を１つずつ変更する
  double target_joint_value = angles::from_degrees(-15.0);
  for (size_t i = 0; i < joint_values.size(); i++) {
    joint_values[i] = target_joint_value;
    move_group_arm.setJointValueTarget(joint_values);
    move_group_arm.move();
  }

  // 初期姿勢に戻す
  move_group_arm.setNamedTarget("l_arm_init_pose");
  move_group_arm.move();

  rclcpp::shutdown();
  return 0;
}
