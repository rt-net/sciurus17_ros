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

#include "sciurus17_examples/waist_jt_control.hpp"

#include <angles/angles.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace sciurus17_examples
{

WaistJtControl::WaistJtControl(const rclcpp::NodeOptions & options)
: Node("waist_control", options)
{
  // 目標角度を購読（ObjectTrackerノードが配信）
  angles_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "target_angles", 10, std::bind(&WaistJtControl::angles_callback, this, _1));

  // JointTrajectoryメッセージを配信（waist_yaw_controllerが購読）
  jt_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/waist_yaw_controller/joint_trajectory", 10);
}

void WaistJtControl::angles_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // 目標位置に到達するまでの時間（JointTrajectoryの仕様）
  const auto TIME_FROM_START = 1ms;

  // 腰の可動範囲（ハードウェア制約）
  const double MAX_YAW_ANGLE = angles::from_degrees(120);
  const double MIN_YAW_ANGLE = angles::from_degrees(-120);

  // メッセージから目標角度を取得（配列長チェック）
  if (msg->data.size() != 2) {
    return;
  }
  auto yaw_angle = msg->data[0];

  // 目標角度を可動範囲内に制限
  yaw_angle = std::clamp(yaw_angle, MIN_YAW_ANGLE, MAX_YAW_ANGLE);

  // JointTrajectoryメッセージを構築
  trajectory_msgs::msg::JointTrajectory jt_msg;
  jt_msg.joint_names.push_back("waist_yaw_joint");

  // 軌道点を設定（現在位置から目標位置へ遷移）
  trajectory_msgs::msg::JointTrajectoryPoint jt_point_msg;
  jt_point_msg.positions.push_back(yaw_angle);
  jt_point_msg.time_from_start = rclcpp::Duration(TIME_FROM_START);
  jt_msg.points.push_back(jt_point_msg);

  jt_publisher_->publish(jt_msg);
}

}  // namespace sciurus17_examples

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(sciurus17_examples::WaistJtControl)
