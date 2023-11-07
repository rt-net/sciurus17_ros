// Copyright 2023 RT Corporation
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
// https://github.com/ros-planning/moveit2/blob/main/moveit_demo_nodes
// /run_move_group/src/run_move_group.cpp

#include <cmath>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("gripper_control");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_two_arm_node = rclcpp::Node::make_shared("move_group_two_arm_node", node_options);
  auto move_group_r_gripper_node = rclcpp::Node::make_shared("move_group_r_gripper_node", node_options);
  auto move_group_l_gripper_node = rclcpp::Node::make_shared("move_group_l_gripper_node", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_two_arm_node);
  executor.add_node(move_group_r_gripper_node);
  executor.add_node(move_group_l_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_two_arm(move_group_two_arm_node, "two_arm_group");
  move_group_two_arm.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_two_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  MoveGroupInterface move_group_r_gripper(move_group_r_gripper_node, "r_gripper_group");
  move_group_r_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_r_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto r_gripper_joint_values = move_group_r_gripper.getCurrentJointValues();

  MoveGroupInterface move_group_l_gripper(move_group_l_gripper_node, "l_gripper_group");
  move_group_l_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_l_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto l_gripper_joint_values = move_group_l_gripper.getCurrentJointValues();

  // SRDFに定義されている"two_arm_init_pose"の姿勢にする
  move_group_two_arm.setNamedTarget("two_arm_init_pose");
  move_group_two_arm.move();

  // 右グリッパ開閉
  r_gripper_joint_values[0] = angles::from_degrees(40);
  move_group_r_gripper.setJointValueTarget(r_gripper_joint_values);
  move_group_r_gripper.move();

  r_gripper_joint_values[0] = angles::from_degrees(0);
  move_group_r_gripper.setJointValueTarget(r_gripper_joint_values);
  move_group_r_gripper.move();

  r_gripper_joint_values[0] = angles::from_degrees(40);
  move_group_r_gripper.setJointValueTarget(r_gripper_joint_values);
  move_group_r_gripper.move();

  r_gripper_joint_values[0] = angles::from_degrees(0);
  move_group_r_gripper.setJointValueTarget(r_gripper_joint_values);
  move_group_r_gripper.move();

  // 左グリッパ開閉
  l_gripper_joint_values[0] = angles::from_degrees(-40);
  move_group_l_gripper.setJointValueTarget(l_gripper_joint_values);
  move_group_l_gripper.move();

  l_gripper_joint_values[0] = angles::from_degrees(0);
  move_group_l_gripper.setJointValueTarget(l_gripper_joint_values);
  move_group_l_gripper.move();

  l_gripper_joint_values[0] = angles::from_degrees(-40);
  move_group_l_gripper.setJointValueTarget(l_gripper_joint_values);
  move_group_l_gripper.move();

  l_gripper_joint_values[0] = angles::from_degrees(0);
  move_group_l_gripper.setJointValueTarget(l_gripper_joint_values);
  move_group_l_gripper.move();

  rclcpp::shutdown();
  return 0;
}