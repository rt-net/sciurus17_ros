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

#include <angles/angles.h>

#include "sciurus17_examples/object_tracker.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace sciurus17_examples
{

ObjectTracker::ObjectTracker(const rclcpp::NodeOptions & options)
: Node("object_tracker", options)
{
  // 30msごとに追従制御を実行するタイマーを作成
  timer_ = this->create_wall_timer(
    30ms, std::bind(&ObjectTracker::tracking, this));

  // コントローラの現在角度を購読
  state_subscription_ =
    this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "/controller_state", 10, std::bind(&ObjectTracker::state_callback, this, _1));

  // ColorDetection2Dノードが配信する物体位置を購読
  object_point_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "target_position", 10, std::bind(&ObjectTracker::point_callback, this, _1));

  // 首/腰の目標角度を配信
  angles_publisher_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("target_angles", 10);
}

void ObjectTracker::state_callback(
  const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
  // 最新のコントローラ状態を保存（tracking()で使用）
  current_angles_msg_ = msg;
}

void ObjectTracker::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // 最新の物体検出位置を保存（tracking()で使用）
  object_point_msg_ = msg;
}

void ObjectTracker::tracking()
{
  // 物体位置が中心からこの閾値以上離れていれば追従動作を行う
  const double POSITION_THRESH = 0.1;

  // 初期姿勢の角度[yaw, pitch]
  const std::vector<double> INITIAL_ANGLES = {0, 0};

  // 首/腰の可動範囲（ハードウェア制約）
  const double MAX_YAW_ANGLE = angles::from_degrees(120);
  const double MIN_YAW_ANGLE = angles::from_degrees(-120);
  const double MAX_PITCH_ANGLE = angles::from_degrees(50);
  const double MIN_PITCH_ANGLE = angles::from_degrees(-75);

  // コントローラの関節数（腰: 1軸、首: 2軸）
  const int WAIST_JOINT_NUM = 1;
  const int NECK_JOINT_NUM = 2;

  // 1回の制御周期での最大角度変化量（急激な動きを防ぐ）
  const double MAX_ANGULAR_DIFF = angles::from_degrees(1.5);

  // 初期姿勢へゆっくり戻る際の角度変化量
  const double RESET_ANGULAR_DIFF = angles::from_degrees(0.5);

  // 物体が検出されなくなってから初期姿勢に戻り始めるまでの猶予時間
  const std::chrono::nanoseconds DETECTION_TIMEOUT = 1s;

  // 追従制御のゲイン（値が大きいほど追従速度が速い）
  const double OPERATION_GAIN = 0.02;

  // 現在物体を追従中かどうかのフラグ
  bool look_object = false;

  // コントローラ状態が届くまで待機
  if (!current_angles_msg_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Waiting controller state.");
    return;
  }

  // コントローラの関節数チェック（腰または首）
  if (current_angles_msg_->feedback.positions.size() != WAIST_JOINT_NUM &&
    current_angles_msg_->feedback.positions.size() != NECK_JOINT_NUM)
  {
    return;
  }

  // 現在の関節角度を取得
  const auto current_angles = current_angles_msg_->feedback.positions;

  // 初回実行時に目標角度を初期化
  if (target_angles_.empty()) {
    target_angles_ = current_angles;
    if (target_angles_.size() == 1) {
      target_angles_.push_back(0);
    }
  }

  // 現在時刻を取得
  auto now = this->get_clock()->now().nanoseconds();

  // 物体が検出されているかタイムアウト判定
  if (object_point_msg_) {
    const auto detected_time = rclcpp::Time(object_point_msg_->header.stamp).nanoseconds();
    const auto POINT_ELAPSED_TIME = now - detected_time;
    // 猶予時間内であれば追従モード
    look_object = POINT_ELAPSED_TIME < DETECTION_TIMEOUT.count();
  }

  // 物体検出中: 追従制御
  if (look_object) {
    // 物体の正規化座標[x, y]を取得（-1.0～1.0の範囲）
    std::vector<double> object_position;
    object_position.push_back(object_point_msg_->point.x);
    object_position.push_back(object_point_msg_->point.y);
    std::vector<double> diff_angles = {0, 0};

    // 物体位置から目標角度の変化量を計算
    for (int i = 0; i < 2; i++) {
      // 物体が中心から閾値以上離れていれば追従動作
      if (std::abs(object_position[i]) > POSITION_THRESH) {
        diff_angles[i] = object_position[i] * OPERATION_GAIN;
        // 急激な動きを防ぐため最大変化量でクランプ
        diff_angles[i] = std::clamp(diff_angles[i], -MAX_ANGULAR_DIFF, MAX_ANGULAR_DIFF);
        target_angles_[i] -= diff_angles[i];
      }
    }
  } else {
    // 物体未検出: ゆっくり初期姿勢へ復帰
    std::vector<double> diff_angles = {0, 0};

    for (int i = 0; i < 2; i++) {
      diff_angles[i] = INITIAL_ANGLES[i] - target_angles_[i];
      if (std::abs(diff_angles[i]) > RESET_ANGULAR_DIFF) {
        // ゆっくり初期姿勢に近づける
        target_angles_[i] += std::copysign(RESET_ANGULAR_DIFF, diff_angles[i]);
      } else {
        // 十分近づいたら初期姿勢に固定
        target_angles_[i] = INITIAL_ANGLES[i];
      }
    }
  }

  // 目標角度を可動範囲内に制限
  target_angles_[0] = std::clamp(target_angles_[0], MIN_YAW_ANGLE, MAX_YAW_ANGLE);
  target_angles_[1] = std::clamp(target_angles_[1], MIN_PITCH_ANGLE, MAX_PITCH_ANGLE);

  // 目標角度をFloat64MultiArrayメッセージとして配信
  std_msgs::msg::Float64MultiArray target_angles_msg;
  target_angles_msg.data.push_back(target_angles_[0]);
  target_angles_msg.data.push_back(target_angles_[1]);
  angles_publisher_->publish(target_angles_msg);
}

}  // namespace sciurus17_examples

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(sciurus17_examples::ObjectTracker)
