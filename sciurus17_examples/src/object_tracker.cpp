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

#include "composition/object_tracker.hpp"

#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>
#include "angles/angles.h"

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace sciurus17_examples
{

ObjectTracker::ObjectTracker(const rclcpp::NodeOptions & options)
: Node("object_tracker", options)
{
  timer_ = this->create_wall_timer(
    20ms, std::bind(&ObjectTracker::tracking, this));

  state_subscription_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "/neck_controller/controller_state", 10, std::bind(&ObjectTracker::state_callback, this, _1));

  object_point_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/object_detected", 10, std::bind(&ObjectTracker::point_callback, this, _1));

  angles_publisher_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("/target_angles", 10);
}

void ObjectTracker::state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
  current_angles_msg_ = msg;
}

void ObjectTracker::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  object_point_msg_ = msg;
}

void ObjectTracker::tracking()
{
  // 追従を開始する物体位置の閾値
  const double THRESH_X = 0.03;
  const double THRESH_Y= 0.03;

  // 首角度の初期値
  const double INITIAL_YAW_ANGLE = 0;
  const double INITIAL_PITCH_ANGLE = 0;

  // 首可動範囲
  const double MAX_YAW_ANGLE = angles::from_degrees(120);
  const double MIN_YAW_ANGLE = angles::from_degrees(-120);
  const double MAX_PITCH_ANGLE = angles::from_degrees(50);
  const double MIN_PITCH_ANGLE = angles::from_degrees(-85);

  // 首角度初期化時の制御角度
  const double RESET_ANGLE_VEL = angles::from_degrees(0.3);

  // 物体が検出されなくなってから初期角度に戻り始めるまでの時間
  const std::chrono::nanoseconds DETECTION_TIMEOUT = 3s;

  // 首角度制御量
  // 値が大きいほど追従速度が速くなる
  const double OPERATION_GAIN_X = 0.05;
  const double OPERATION_GAIN_Y = 0.05;

  // 追従動作開始フラグ
  bool look_object = false;

  // 現在の首角度を取得
  if (!current_angles_msg_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Wating controller state.");
    return;
  }
  if (current_angles_msg_->feedback.positions.size() != 2) {
    return;
  }
  const auto current_angles = current_angles_msg_->feedback.positions;
  if (target_angles_.empty()) {
    target_angles_ = current_angles;
  }

  // 現在時刻
  auto now = this->get_clock()->now().nanoseconds();

  if (object_point_msg_) {
    // 物体を検出したとき追従動作開始フラグをtrueにする
    const auto detected_time = rclcpp::Time(object_point_msg_->header.stamp).nanoseconds();
    const auto POINT_ELAPSED_TIME = now - detected_time;
    look_object = POINT_ELAPSED_TIME < DETECTION_TIMEOUT.count();
  }

  // 物体が検出されたら追従を行う
  if (look_object) {
    // 物体検出位置を取得
    const auto object_position = object_point_msg_->point;

    // 追従動作のための首角度を計算
    if (std::abs(object_position.x) > THRESH_X) {
      target_angles_[0] -= object_position.x * OPERATION_GAIN_X;
    }
    if (std::abs(object_position.y) > THRESH_Y) {
      target_angles_[1] -= object_position.y * OPERATION_GAIN_Y;
    }
  } else {
    // ゆっくりと初期角度へ戻る
    auto diff_yaw_angle = INITIAL_YAW_ANGLE - target_angles_[0];
    if (std::abs(diff_yaw_angle) > RESET_ANGLE_VEL) {
      target_angles_[0] += std::copysign(RESET_ANGLE_VEL, diff_yaw_angle);
    } else {
      target_angles_[0] = INITIAL_YAW_ANGLE;
    }

    auto diff_pitch_angle = INITIAL_PITCH_ANGLE - target_angles_[1];
    if (std::abs(diff_pitch_angle) > RESET_ANGLE_VEL) {
      target_angles_[1] += std::copysign(RESET_ANGLE_VEL, diff_pitch_angle);
    } else {
      target_angles_[1] = INITIAL_PITCH_ANGLE;
    }
  }

  // 目標首角度を制限角度内に収める
  target_angles_[0] = std::clamp(target_angles_[0], MIN_YAW_ANGLE, MAX_YAW_ANGLE);
  target_angles_[1] = std::clamp(target_angles_[1], MIN_PITCH_ANGLE, MAX_PITCH_ANGLE);

  // 目標角度に首を動かす
  std_msgs::msg::Float64MultiArray target_angles_msg;
  target_angles_msg.data.push_back(target_angles_[0]);
  target_angles_msg.data.push_back(target_angles_[1]);
  angles_publisher_->publish(target_angles_msg);
}

}  // namespace sciurus17_examples

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sciurus17_examples::ObjectTracker)