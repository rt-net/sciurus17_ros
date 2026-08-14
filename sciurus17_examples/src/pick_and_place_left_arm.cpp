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
// https://github.com/ros-planning/moveit2_tutorials/blob/humble/doc/
// examples/move_group_interface/src/move_group_interface_tutorial.cpp

#include <cmath>
#include <thread>

#include <angles/angles.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pose_presets.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlace
{
public:
  // グリッパの開閉角度
  inline static const double GRIPPER_OPEN = angles::from_degrees(-40.0);
  inline static const double GRIPPER_GRASP = angles::from_degrees(-20.0);
  inline static const double GRIPPER_CLOSE = 0.0;

  // ノードを受け取り、アーム・グリッパのMoveGroupInterfaceを初期化する
  explicit PickAndPlace(
    rclcpp::Node::SharedPtr arm_node,
    rclcpp::Node::SharedPtr gripper_node)
  {
    l_arm_group_ = std::make_shared<MoveGroupInterface>(arm_node, "l_arm_group");
    l_arm_group_->setMaxVelocityScalingFactor(0.1);  // Set 0.0 ~ 1.0
    l_arm_group_->setMaxAccelerationScalingFactor(0.1);  // Set 0.0 ~ 1.0

    l_gripper_group_ = std::make_shared<MoveGroupInterface>(gripper_node, "l_gripper_group");
  }

  // アームを目標位置・姿勢（Pose）に動かす
  void move_arm_to_pose(const geometry_msgs::msg::Pose & pose)
  {
    l_arm_group_->setPoseTarget(pose);
    l_arm_group_->move();
  }

  // アームを目標位置（x, y, z [m]）に動かす（グリッパは下向き固定）
  void control_arm(const double x, const double y, const double z)
  {
    move_arm_to_pose(pose_presets::left_arm_downward(x, y, z));
  }

  // SRDFに定義された姿勢名でアームを動かす
  void move_arm_to_named_pose(const std::string & name)
  {
    l_arm_group_->setNamedTarget(name);
    l_arm_group_->move();
  }

  // グリッパを角度[rad]を指定して開閉する
  void move_gripper_angle(const double angle)
  {
    auto joint_values = l_gripper_group_->getCurrentJointValues();
    joint_values[0] = angle;
    l_gripper_group_->setJointValueTarget(joint_values);
    l_gripper_group_->move();
  }

private:
  std::shared_ptr<MoveGroupInterface> l_arm_group_;
  std::shared_ptr<MoveGroupInterface> l_gripper_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  PickAndPlace controller(move_group_arm_node, move_group_gripper_node);

  // アプローチ・退避時の高さ
  const double LIFTING_HEIGHT = 0.25;

  // 掴む位置（ピック位置）のXYZ[m]
  const double PICK_X = 0.25;
  const double PICK_Y = 0.0;
  const double PICK_Z = 0.12;

  // 置く位置（プレース位置）のXYZ[m]
  const double PLACE_X = 0.35;
  const double PLACE_Y = 0.0;
  const double PLACE_Z = 0.12;

  // 初期化動作
  controller.move_arm_to_named_pose("l_arm_init_pose");
  // 何かを掴んでいた時のために開く
  controller.move_gripper_angle(PickAndPlace::GRIPPER_OPEN);

  // ピック動作（掴みに行く）
  // 物体の上に腕を伸ばす
  controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT);
  // アプローチ
  controller.control_arm(PICK_X, PICK_Y, PICK_Z);
  // 掴む
  controller.move_gripper_angle(PickAndPlace::GRIPPER_GRASP);
  // 持ち上げる
  controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT);

  // プレース動作（移動して置く）
  // 移動する
  controller.control_arm(PLACE_X, PLACE_Y, LIFTING_HEIGHT);
  // 下ろす
  controller.control_arm(PLACE_X, PLACE_Y, PLACE_Z);
  // 離す
  controller.move_gripper_angle(PickAndPlace::GRIPPER_OPEN);
  // 持ち上げる
  controller.control_arm(PLACE_X, PLACE_Y, LIFTING_HEIGHT);

  // 終了動作
  controller.move_arm_to_named_pose("l_arm_init_pose");
  controller.move_gripper_angle(PickAndPlace::GRIPPER_CLOSE);

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
