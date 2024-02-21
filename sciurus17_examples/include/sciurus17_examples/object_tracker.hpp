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

#ifndef SCIURUS17_EXAMPLES__OBJECT_TRACKER_HPP_
#define SCIURUS17_EXAMPLES__OBJECT_TRACKER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace sciurus17_examples
{

class ObjectTracker : public rclcpp::Node
{
public:
  explicit ObjectTracker(const rclcpp::NodeOptions & options);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr object_point_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angles_publisher_;
  control_msgs::msg::JointTrajectoryControllerState::SharedPtr current_angles_msg_;
  geometry_msgs::msg::PointStamped::SharedPtr object_point_msg_;
  std::vector<double> target_angles_;

  void state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void tracking();
};

}  // namespace sciurus17_examples

#endif  // SCIURUS17_EXAMPLES__OBJECT_TRACKER_HPP_
