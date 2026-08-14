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
// /5c15da709e9ea8529b54b313dc570f164f9a713e/doc/examples/subframes
// /src/subframes_tutorial.cpp

#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PoseGroupstate
{
public:
  // ノードを受け取り、アームのMoveGroupInterfaceを初期化する
  explicit PoseGroupstate(rclcpp::Node::SharedPtr node)
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(node, "two_arm_group");
    move_group_arm_->setMaxVelocityScalingFactor(0.1);      // Set 0.0 ~ 1.0
    move_group_arm_->setMaxAccelerationScalingFactor(0.1);  // Set 0.0 ~ 1.0
  }

  // SRDFに定義された姿勢名でアームを動かす
  void move_arm_to_named_pose(const std::string & name)
  {
    move_group_arm_->setNamedTarget(name);
    move_group_arm_->move();
  }

private:
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pose_groupstate", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([node]() { rclcpp::spin(node); });

  PoseGroupstate controller(node);

  // SRDFに定義された名前付き姿勢を順番に動かす
  controller.move_arm_to_named_pose("two_arm_init_pose");
  controller.move_arm_to_named_pose("two_arm_push_forward_pose");
  controller.move_arm_to_named_pose("two_arm_init_pose");

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
