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
// https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html

#include "composition/neck_jt_control.hpp"

#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>
#include "angles/angles.h"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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

  this->client_ptr_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    this,
    "neck_controller/follow_joint_trajectory");

  if(!this->client_ptr_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }
}

void NeckJtControl::angles_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  const double TIME_FROM_START = 1.0e-9;

  if (!has_result_) {
    return;
  }

  if (msg->data.size() != 2) {
    return;
  }
  auto yaw_angle = msg->data[0];
  auto pitch_angle = msg->data[1];

  auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names.push_back("neck_yaw_joint");
  goal_msg.trajectory.joint_names.push_back("neck_pitch_joint");

  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.push_back(yaw_angle);
  trajectory_point_msg.positions.push_back(pitch_angle);
  trajectory_point_msg.time_from_start = rclcpp::Duration::from_seconds(TIME_FROM_START);
  goal_msg.trajectory.points.push_back(trajectory_point_msg);

  auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.result_callback =std::bind(&NeckJtControl::result_callback, this, _1);

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  has_result_ = false;
}

void NeckJtControl::result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {
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
  has_result_ = true;
}

}  // namespace sciurus17_examples

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sciurus17_examples::NeckJtControl)