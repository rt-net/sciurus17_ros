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

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace sciurus17_examples
{

// 検出した物体を追従するコンポーネントノード
// ColorDetection2Dノードから物体位置を受け取り、首/腰の目標角度を計算して配信
// 30msタイマーで追従制御ループを実行
class ObjectTracker : public rclcpp::Node
{
public:
  explicit ObjectTracker(const rclcpp::NodeOptions & options);

private:
  // 30msごとに追従制御を実行するタイマー
  rclcpp::TimerBase::SharedPtr timer_;

  // 首/腰コントローラの現在角度を購読するサブスクライバ
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    state_subscription_;

  // 物体の正規化座標を購読するサブスクライバ（ColorDetection2Dが配信）
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr object_point_subscription_;

  // 首/腰の目標角度を配信するパブリッシャ（NeckJtControl, WaistJtControlが購読）
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angles_publisher_;

  // 最新の関節角度を保持
  control_msgs::msg::JointTrajectoryControllerState::SharedPtr current_angles_msg_;

  // 最新の物体検出位置を保持
  geometry_msgs::msg::PointStamped::SharedPtr object_point_msg_;

  // 現在の目標角度[yaw, pitch]を保持
  std::vector<double> target_angles_;

  // コントローラ状態を受信したときに呼ばれるコールバック関数
  void state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);

  // 物体位置を受信したときに呼ばれるコールバック関数
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  // タイマーごとに呼ばれる追従制御ループ
  // 物体位置に基づいて目標角度を更新し配信
  void tracking();

  // 物体の正規化座標から目標角度（追従方向）を更新する
  void update_target_angles_for_tracking(const std::vector<double> & object_position);

  // 目標角度を初期姿勢へゆっくり近づける
  void update_target_angles_for_reset();
};

}  // namespace sciurus17_examples

#endif  // SCIURUS17_EXAMPLES__OBJECT_TRACKER_HPP_
