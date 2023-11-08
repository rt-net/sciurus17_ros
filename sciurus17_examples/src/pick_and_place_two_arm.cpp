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
// https://github.com/ros-planning/moveit2_tutorials/blob
// /a547cf49ff7d1fe16a93dfe020c6027bcb035b51/doc/move_group_interface
// /src/move_group_interface_tutorial.cpp

#include <cmath>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pose_presets.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_two_arm");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_r_gripper_node =
    rclcpp::Node::make_shared("move_group_r_gripper_node", node_options);
  auto move_group_l_gripper_node =
    rclcpp::Node::make_shared("move_group_l_gripper_node", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_r_gripper_node);
  executor.add_node(move_group_l_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_arm(move_group_arm_node, "two_arm_group");
  move_group_arm.setMaxVelocityScalingFactor(0.2);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(0.2);  // Set 0.0 ~ 1.0

  MoveGroupInterface move_group_r_gripper(move_group_r_gripper_node, "r_gripper_group");
  move_group_r_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_r_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto r_gripper_joint_values = move_group_r_gripper.getCurrentJointValues();

  MoveGroupInterface move_group_l_gripper(move_group_l_gripper_node, "l_gripper_group");
  move_group_l_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_l_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto l_gripper_joint_values = move_group_l_gripper.getCurrentJointValues();

  // double GRIPPER_DEFAULT = 0.0;
  // double GRIPPER_OPEN = angles::from_degrees(40.0);
  // double GRIPPER_CLOSE = angles::from_degrees(20.0);

  // SRDFに定義されている"two_arm_init_pose"の姿勢にする
  move_group_arm.setNamedTarget("two_arm_init_pose");
  move_group_arm.move();

  // 何かを掴んでいた時のためにハンドを開く
  /*
  r_gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_r_gripper.setJointValueTarget(r_gripper_joint_values);
  move_group_r_gripper.move();

  l_gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_l_gripper.setJointValueTarget(l_gripper_joint_values);
  move_group_l_gripper.move();

  r_gripper_joint_values[0] = GRIPPER_CLOSE;
  move_group_r_gripper.setJointValueTarget(r_gripper_joint_values);
  move_group_r_gripper.move();

  l_gripper_joint_values[0] = GRIPPER_CLOSE;
  move_group_l_gripper.setJointValueTarget(l_gripper_joint_values);
  move_group_l_gripper.move();
  */
  // 可動範囲を制限する
  /*
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "arm_constraints";

  moveit_msgs::msg::JointConstraint joint_constraint;
  joint_constraint.joint_name = "waist_yaw_joint";
  joint_constraint.position = 0.0;
  joint_constraint.tolerance_above = angles::from_degrees(45);
  joint_constraint.tolerance_below = angles::from_degrees(45);
  joint_constraint.weight = 1.0;
  constraints.joint_constraints.push_back(joint_constraint);

  move_group_arm.setPathConstraints(constraints);
  */

  // 掴む準備をする
  pose_presets::PosePresets target_pose;
  move_group_arm.setPoseTarget(target_pose.right_downward(0.18, -0.25, 0.2), "r_link7");
  move_group_arm.setPoseTarget(target_pose.right_downward(0.18, 0.25, 0.2), "l_link7");
  move_group_arm.move();

  /*
  r_gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_r_gripper.setJointValueTarget(r_gripper_joint_values);
  move_group_r_gripper.move();

  l_gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_l_gripper.setJointValueTarget(l_gripper_joint_values);
  move_group_l_gripper.move();
  */
  // 手を内向きにする
  move_group_arm.setPoseTarget(target_pose.generate_pose(0.18, -0.25, 0.08, angles::from_degrees(180), 0.0, 0.0), "r_link7");
  move_group_arm.setPoseTarget(target_pose.generate_pose(0.18, 0.25, 0.08, angles::from_degrees(-180), 0.0, 0.0), "l_link7");
  move_group_arm.move();
  // 掴みに行く準備
  // 軌道成約の追加
  // 手を平行に寄せる
  move_group_arm.setPoseTarget(target_pose.generate_pose(0.18, -0.10, 0.08, angles::from_degrees(180), 0.0, 0.0), "r_link7");
  move_group_arm.setPoseTarget(target_pose.generate_pose(0.18, 0.10, 0.08, angles::from_degrees(-180), 0.0, 0.0), "l_link7");
  move_group_arm.move();
  // 掴みに行く
  move_group_arm.setPoseTarget(target_pose.generate_pose(0.18, -0.08, 0.08, angles::from_degrees(180), 0.0, 0.0), "r_link7");
  move_group_arm.setPoseTarget(target_pose.generate_pose(0.18, 0.08, 0.08, angles::from_degrees(-180), 0.0, 0.0), "l_link7");
  move_group_arm.move();

  // ハンドを閉じる

  // 前へ押し出す
  move_group_arm.setPoseTarget(target_pose.generate_pose(0.35, -0.09, 0.08, angles::from_degrees(180), 0.0, 0.0), "r_link7");
  move_group_arm.setPoseTarget(target_pose.generate_pose(0.35, 0.09, 0.08, angles::from_degrees(-180), 0.0, 0.0), "l_link7");
  move_group_arm.move();

  // ハンドを開く

  // 軌道成約の解除

  // 離す
  move_group_arm.setPoseTarget(target_pose.right_downward(0.28, -0.20, 0.12), "r_link7");
  move_group_arm.setPoseTarget(target_pose.right_downward(0.28, 0.20, 0.12), "l_link7");
  move_group_arm.move();

  // ハンドを閉じる

  // 腕を持ち上げてターゲットから離れる
  move_group_arm.setPoseTarget(target_pose.right_downward(0.32, -0.25, 0.30), "r_link7");
  move_group_arm.setPoseTarget(target_pose.right_downward(0.32, 0.25, 0.30), "l_link7");
  move_group_arm.move();

  // SRDFに定義されている"two_arm_init_pose"の姿勢にする
  move_group_arm.setNamedTarget("two_arm_init_pose");
  move_group_arm.move();

  rclcpp::shutdown();
  return 0;
}
