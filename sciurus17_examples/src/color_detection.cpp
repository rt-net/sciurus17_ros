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

#include "sciurus17_examples/color_detection.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
using std::placeholders::_1;

namespace sciurus17_examples
{

ColorDetection::ColorDetection(const rclcpp::NodeOptions & options)
: Node("color_detection", options)
{
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/head_camera/color/image_raw", 10, std::bind(&ColorDetection::image_callback, this, _1));

  image_thresholded_publisher_ =
    this->create_publisher<sensor_msgs::msg::Image>("image_thresholded", 10);

  object_point_publisher_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>("target_position", 10);
}

void ColorDetection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // オレンジ色の物体を検出するようにHSVの範囲を設定
  const int LOW_H = 5, HIGH_H = 20;
  const int LOW_S = 120, HIGH_S = 255;
  const int LOW_V = 120, HIGH_V = 255;

  auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);

  // 画像をRGBからHSVに変換
  cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_RGB2HSV);

  // 画像処理用の変数を用意
  cv::Mat img_thresholded;

  // 画像の二値化
  cv::inRange(
    cv_img->image,
    cv::Scalar(LOW_H, LOW_S, LOW_V),
    cv::Scalar(HIGH_H, HIGH_S, HIGH_V),
    img_thresholded);

  // ノイズ除去の処理
  cv::morphologyEx(
    img_thresholded,
    img_thresholded,
    cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

  // 穴埋めの処理
  cv::morphologyEx(
    img_thresholded,
    img_thresholded,
    cv::MORPH_CLOSE,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

  // 閾値による二値化画像を配信
  sensor_msgs::msg::Image::SharedPtr img_thresholded_msg =
    cv_bridge::CvImage(msg->header, "mono8", img_thresholded).toImageMsg();
  image_thresholded_publisher_->publish(*img_thresholded_msg);

  // 画像の検出領域におけるモーメントを計算
  cv::Moments moment = moments(img_thresholded);
  double d_m01 = moment.m01;
  double d_m10 = moment.m10;
  double d_area = moment.m00;

  // 検出領域のピクセル数が100000より大きい場合
  if (d_area > 100000) {
    // 画像座標系における物体検出位置（2D）
    cv::Point2d object_point;
    object_point.x = d_m10 / d_area;
    object_point.y = d_m01 / d_area;

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Detect at" << object_point << ".");

    // 画像の中心を原点とした検出位置に変換
    cv::Point2d translated_object_point;
    translated_object_point.x = object_point.x - msg->width / 2.0;
    translated_object_point.y = object_point.y - msg->height / 2.0;

    // 検出位置を-1.0 ~ 1.0に正規化
    cv::Point2d normalized_object_point_;
    if (msg->width != 0 && msg->height != 0) {
      normalized_object_point_.x = translated_object_point.x / (msg->width / 2.0);
      normalized_object_point_.y = translated_object_point.y / (msg->height / 2.0);
    }

    // 検出位置を配信
    geometry_msgs::msg::PointStamped object_point_msg;
    object_point_msg.header = msg->header;
    object_point_msg.point.x = normalized_object_point_.x;
    object_point_msg.point.y = normalized_object_point_.y;
    object_point_publisher_->publish(object_point_msg);
  }
}

}  // namespace sciurus17_examples

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sciurus17_examples::ColorDetection)
