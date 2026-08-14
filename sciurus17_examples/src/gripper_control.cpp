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

#include <angles/angles.h>

#include <cmath>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class GripperControl
{
public:
  // ノードを受け取り、アーム・両グリッパのMoveGroupInterfaceを初期化する
  explicit GripperControl(rclcpp::Node::SharedPtr node)
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(node, "two_arm_group");
    move_group_arm_->setMaxVelocityScalingFactor(0.1);      // Set 0.0 ~ 1.0
    move_group_arm_->setMaxAccelerationScalingFactor(0.1);  // Set 0.0 ~ 1.0

    move_group_r_gripper_ = std::make_shared<MoveGroupInterface>(node, "r_gripper_group");
    move_group_l_gripper_ = std::make_shared<MoveGroupInterface>(node, "l_gripper_group");
  }

  // SRDFに定義された姿勢名でアームを動かす
  void move_arm_to_named_pose(const std::string & name)
  {
    move_group_arm_->setNamedTarget(name);
    move_group_arm_->move();
  }

  // 右グリッパを角度[rad]を指定して開閉する
  void move_r_gripper_angle(const double angle)
  {
    auto joint_values = move_group_r_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_r_gripper_->setJointValueTarget(joint_values);
    move_group_r_gripper_->move();
  }

  // 左グリッパを角度[rad]を指定して開閉する
  void move_l_gripper_angle(const double angle)
  {
    auto joint_values = move_group_l_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_l_gripper_->setJointValueTarget(joint_values);
    move_group_l_gripper_->move();
  }

private:
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_r_gripper_;
  std::shared_ptr<MoveGroupInterface> move_group_l_gripper_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("gripper_control", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([node]() { rclcpp::spin(node); });

  GripperControl controller(node);

  const double R_GRIPPER_CLOSE = 0.0;
  const double R_GRIPPER_OPEN = angles::from_degrees(40.0);
  const double L_GRIPPER_CLOSE = 0.0;
  const double L_GRIPPER_OPEN = angles::from_degrees(-40.0);

  // two_arm_init_poseの姿勢にする
  controller.move_arm_to_named_pose("two_arm_init_pose");

  // 右グリッパを2回開閉する
  for (int i = 0; i < 2; i++) {
    controller.move_r_gripper_angle(R_GRIPPER_OPEN);
    controller.move_r_gripper_angle(R_GRIPPER_CLOSE);
  }

  // 左グリッパを2回開閉する
  for (int i = 0; i < 2; i++) {
    controller.move_l_gripper_angle(L_GRIPPER_OPEN);
    controller.move_l_gripper_angle(L_GRIPPER_CLOSE);
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
