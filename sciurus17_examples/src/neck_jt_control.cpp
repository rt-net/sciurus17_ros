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

#include "sciurus17_examples/neck_jt_control.hpp"

#include "angles/angles.h"

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace sciurus17_examples
{

NeckJtControl::NeckJtControl(const rclcpp::NodeOptions & options)
: Node("neck_control", options)
{
  angles_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/target_angles", 10, std::bind(&NeckJtControl::angles_callback, this, _1));

  jt_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "neck_controller/joint_trajectory", 10);
}

void NeckJtControl::angles_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // 動作時間
  const double TIME_FROM_START = 1.0e-9;
  // 首可動範囲
  const double MAX_YAW_ANGLE = angles::from_degrees(120);
  const double MIN_YAW_ANGLE = angles::from_degrees(-120);
  const double MAX_PITCH_ANGLE = angles::from_degrees(50);
  const double MIN_PITCH_ANGLE = angles::from_degrees(-75);

  // 角度指令値取得
  if (msg->data.size() != 2) {
    return;
  }
  auto yaw_angle = msg->data[0];
  auto pitch_angle = msg->data[1];

  // 角度指令値を可動範囲内にする
  yaw_angle = std::clamp(yaw_angle, MIN_YAW_ANGLE, MAX_YAW_ANGLE);
  pitch_angle = std::clamp(pitch_angle, MIN_PITCH_ANGLE, MAX_PITCH_ANGLE);

  // joint名設定
  trajectory_msgs::msg::JointTrajectory jt_msg;
  jt_msg.joint_names.push_back("neck_yaw_joint");
  jt_msg.joint_names.push_back("neck_pitch_joint");

  // 角度指令値設定
  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.push_back(yaw_angle);
  trajectory_point_msg.positions.push_back(pitch_angle);
  trajectory_point_msg.time_from_start = rclcpp::Duration::from_seconds(TIME_FROM_START);
  jt_msg.points.push_back(trajectory_point_msg);

  jt_publisher_->publish(jt_msg);
}

}  // namespace sciurus17_examples

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sciurus17_examples::NeckJtControl)
