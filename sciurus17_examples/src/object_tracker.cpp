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

#include "sciurus17_examples/object_tracker.hpp"

#include "angles/angles.h"

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
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
    30ms, std::bind(&ObjectTracker::tracking, this));

  state_subscription_ =
    this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "/neck_controller/controller_state", 10, std::bind(&ObjectTracker::state_callback, this, _1));

  object_point_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "target_position", 10, std::bind(&ObjectTracker::point_callback, this, _1));

  angles_publisher_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("target_angles", 10);
}

void ObjectTracker::state_callback(
  const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
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
  const double POSITION_THRESH = 0.1;

  // 角度指令値の初期値
  const std::vector<double> INITIAL_ANGLES = {0, 0};

  // 可動範囲
  const double MAX_YAW_ANGLE = angles::from_degrees(120);
  const double MIN_YAW_ANGLE = angles::from_degrees(-120);
  const double MAX_PITCH_ANGLE = angles::from_degrees(50);
  const double MIN_PITCH_ANGLE = angles::from_degrees(-75);

  // 最大角度変化量
  const double MAX_ANGULAR_DIFF = angles::from_degrees(1.5);

  // 初期角度へ戻る際の角度変化量
  const double RESET_ANGULAR_DIFF = angles::from_degrees(0.5);

  // 物体が検出されなくなってから初期角度に戻り始めるまでの時間
  const std::chrono::nanoseconds DETECTION_TIMEOUT = 1s;

  // 追従速度ゲイン
  // 値が大きいほど追従速度が速くなる
  const double OPERATION_GAIN = 0.02;

  // 追従フラグ
  bool look_object = false;

  // 現在の関節角度を取得
  if (!current_angles_msg_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Wating controller state.");
    return;
  }
  if (current_angles_msg_->feedback.positions.size() != 1 &&
    current_angles_msg_->feedback.positions.size() != 2)
  {
    return;
  }
  const auto current_angles = current_angles_msg_->feedback.positions;
  if (target_angles_.empty()) {
    target_angles_ = current_angles;
    if (target_angles_.size() == 1) {
      target_angles_.push_back(0);
    }
  }

  // 現在時刻取得
  auto now = this->get_clock()->now().nanoseconds();

  if (object_point_msg_) {
    // タイムアウト時間以内に物体を検出したとき追従フラグをtrueにする
    const auto detected_time = rclcpp::Time(object_point_msg_->header.stamp).nanoseconds();
    const auto POINT_ELAPSED_TIME = now - detected_time;
    look_object = POINT_ELAPSED_TIME < DETECTION_TIMEOUT.count();
  }

  // 物体が検出されたら追従を行う
  if (look_object) {
    // 物体検出位置を取得
    std::vector<double> object_position;
    object_position.push_back(object_point_msg_->point.x);
    object_position.push_back(object_point_msg_->point.y);
    std::vector<double> diff_angles = {0, 0};

    // 追従動作のための角度を計算
    for (int i = 0; i < 2; i++) {
      if (std::abs(object_position[i]) > POSITION_THRESH) {
        diff_angles[i] = object_position[i] * OPERATION_GAIN;
        diff_angles[i] = std::clamp(diff_angles[i], -MAX_ANGULAR_DIFF, MAX_ANGULAR_DIFF);
        target_angles_[i] -= diff_angles[i];
      }
    }
  } else {
    // ゆっくりと初期角度へ戻る
    std::vector<double> diff_angles = {0, 0};

    for (int i = 0; i < 2; i++) {
      diff_angles[i] = INITIAL_ANGLES[i] - target_angles_[i];
      if (std::abs(diff_angles[i]) > RESET_ANGULAR_DIFF) {
        target_angles_[i] += std::copysign(RESET_ANGULAR_DIFF, diff_angles[i]);
      } else {
        target_angles_[i] = INITIAL_ANGLES[i];
      }
    }
  }

  // 角度指令値を可動範囲内にする
  target_angles_[0] = std::clamp(target_angles_[0], MIN_YAW_ANGLE, MAX_YAW_ANGLE);
  target_angles_[1] = std::clamp(target_angles_[1], MIN_PITCH_ANGLE, MAX_PITCH_ANGLE);

  // 角度指令値を配信する
  std_msgs::msg::Float64MultiArray target_angles_msg;
  target_angles_msg.data.push_back(target_angles_[0]);
  target_angles_msg.data.push_back(target_angles_[1]);
  angles_publisher_->publish(target_angles_msg);
}

}  // namespace sciurus17_examples

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sciurus17_examples::ObjectTracker)
