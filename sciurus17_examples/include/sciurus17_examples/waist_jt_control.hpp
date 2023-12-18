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

#ifndef SCIURUS17_EXAMPLES__WAIST_JT_CONTROL_HPP_
#define SCIURUS17_EXAMPLES__WAIST_JT_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using GoalHandleJt = rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;

namespace sciurus17_examples
{

class WaistJtControl : public rclcpp::Node
{
public:
  explicit WaistJtControl(const rclcpp::NodeOptions & options);

private:
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client_ptr_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angles_subscription_;
  bool has_result_ = true;

  void angles_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void result_callback(const GoalHandleJt::WrappedResult & result);
};

}  // namespace sciurus17_examples

#endif  // SCIURUS17_EXAMPLES__WAIST_JT_CONTROL_HPP_
