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

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class WaistControl
{
public:
  // ノードを受け取り、腰のMoveGroupInterfaceを初期化する
  explicit WaistControl(rclcpp::Node::SharedPtr node)
  {
    move_group_waist_ = std::make_shared<MoveGroupInterface>(node, "waist_group");
    move_group_waist_->setMaxVelocityScalingFactor(0.1);      // Set 0.0 ~ 1.0
    move_group_waist_->setMaxAccelerationScalingFactor(0.1);  // Set 0.0 ~ 1.0
  }

  // SRDFに定義された姿勢名で腰を動かす
  void move_to_named_pose(const std::string & name)
  {
    move_group_waist_->setNamedTarget(name);
    move_group_waist_->move();
  }

  // 各ジョイント角度[rad]を指定して腰を動かす
  void move_joint_values(const std::vector<double> & joint_values)
  {
    move_group_waist_->setJointValueTarget(joint_values);
    move_group_waist_->move();
  }

  // 腰の現在のジョイント角度を取得する
  std::vector<double> get_current_joint_values()
  {
    return move_group_waist_->getCurrentJointValues();
  }

private:
  std::shared_ptr<MoveGroupInterface> move_group_waist_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("waist_control", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([node]() {rclcpp::spin(node);});

  WaistControl controller(node);

  // SRDFに定義されている"waist_init_pose"の姿勢にする
  controller.move_to_named_pose("waist_init_pose");

  // 現在角度をベースに、目標角度を作成する
  auto joint_values = controller.get_current_joint_values();

  // 腰を左に向ける
  joint_values[0] = angles::from_degrees(45.0);
  controller.move_joint_values(joint_values);

  // 腰を右に向ける
  joint_values[0] = angles::from_degrees(-45.0);
  controller.move_joint_values(joint_values);

  // "waist_init_pose"に戻す
  controller.move_to_named_pose("waist_init_pose");

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
