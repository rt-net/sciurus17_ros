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

#ifndef SCIURUS17_EXAMPLES__COLOR_DETECTION_2D_HPP_
#define SCIURUS17_EXAMPLES__COLOR_DETECTION_2D_HPP_

#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace sciurus17_examples
{

// 画像からオレンジ色の物体を検出し、その位置を配信するコンポーネントノード
// ROS 2のコンポーネントとして登録され、別プロセスから動的にロード可能
class ColorDetection2D : public rclcpp::Node
{
public:
  explicit ColorDetection2D(const rclcpp::NodeOptions & options);

private:
  // 画像トピック（/image_raw）を購読するサブスクライバ
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  // 検出結果を描画した画像を配信するパブリッシャ
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_annotated_publisher_;

  // 検出した物体の正規化座標（-1.0～1.0）を配信するパブリッシャ
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr object_point_publisher_;

  // 画像トピックを受信したときに呼ばれるコールバック関数
  // HSV色空間で物体を検出し、画像中心を原点とした正規化座標を計算する
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

}  // namespace sciurus17_examples

#endif  // SCIURUS17_EXAMPLES__COLOR_DETECTION_2D_HPP_
