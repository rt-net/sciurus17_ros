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

#include "angles/angles.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using GoalHandleJt = rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;

namespace sciurus17_examples
{

WaistJtControl::WaistJtControl(const rclcpp::NodeOptions & options)
: Node("waist_control", options)
{
  angles_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/target_angles", 10, std::bind(&WaistJtControl::angles_callback, this, _1));

  this->client_ptr_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    this,
    "waist_yaw_controller/follow_joint_trajectory");

  if (!this->client_ptr_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }
}

void WaistJtControl::angles_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // 動作時間
  const double TIME_FROM_START = 1.0e-9;
  // 首可動範囲
  const double MAX_YAW_ANGLE = angles::from_degrees(120);
  const double MIN_YAW_ANGLE = angles::from_degrees(-120);
  const double MAX_PITCH_ANGLE = angles::from_degrees(50);
  const double MIN_PITCH_ANGLE = angles::from_degrees(-75);


  // 動作完了していない場合はgoalを配信しない
  if (!has_result_) {
    return;
  }

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
  auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names.push_back("waist_yaw_joint");

  // 角度指令値設定
  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.push_back(yaw_angle);
  trajectory_point_msg.time_from_start = rclcpp::Duration::from_seconds(TIME_FROM_START);
  goal_msg.trajectory.points.push_back(trajectory_point_msg);

  auto send_goal_options =
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&WaistJtControl::result_callback, this, _1);

  // 角度指令値配信
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  has_result_ = false;
}

void WaistJtControl::result_callback(
  const GoalHandleJt::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  // 動作完了フラグをtrueにする
  has_result_ = true;
}

}  // namespace sciurus17_examples

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sciurus17_examples::WaistJtControl)
