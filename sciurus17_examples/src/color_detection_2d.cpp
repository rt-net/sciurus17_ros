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
// https://docs.opencv.org/4.5.4/d0/d49/tutorial_moments.html

#include "sciurus17_examples/color_detection_2d.hpp"

#include <cv_bridge/cv_bridge.hpp>

using std::placeholders::_1;

namespace sciurus17_examples
{

ColorDetection2D::ColorDetection2D(const rclcpp::NodeOptions & options)
: Node("color_detection_2d", options)
{
  // カメラ画像を購読（QoS: 10）
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&ColorDetection2D::image_callback, this, _1));

  // 検出結果を描画した画像を配信
  image_annotated_publisher_ =
    this->create_publisher<sensor_msgs::msg::Image>("image_annotated", 10);

  // 物体の正規化座標を配信（画像追従ノードが購読する）
  object_point_publisher_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>("target_position", 10);
}

void ColorDetection2D::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // オレンジ色の物体を検出するようにHSV色空間の範囲を設定
  const int LOW_H = 5, HIGH_H = 20;
  const int LOW_S = 120, HIGH_S = 255;
  const int LOW_V = 120, HIGH_V = 255;

  // 画像全体の1%以上の大きさで映った物体を検出対象とする
  const auto MIN_OBJECT_SIZE = msg->width * msg->height * 0.01;

  // ROS 2画像メッセージをOpenCV形式に変換（ゼロコピー共有）
  auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);

  // RGB画像をHSV色空間に変換（色による物体検出を行うため）
  cv::Mat img_hsv;
  cv::cvtColor(cv_img->image, img_hsv, cv::COLOR_RGB2HSV);

  // 指定したHSV範囲内のピクセルを白（255）、それ以外を黒（0）にする二値化処理
  cv::Mat img_thresholded;
  cv::inRange(
    img_hsv, cv::Scalar(LOW_H, LOW_S, LOW_V), cv::Scalar(HIGH_H, HIGH_S, HIGH_V), img_thresholded);

  // モルフォロジー演算でノイズを除去（オープニング処理）
  cv::morphologyEx(
    img_thresholded, img_thresholded, cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

  // モルフォロジー演算で小さな穴を埋める（クロージング処理）
  cv::morphologyEx(
    img_thresholded, img_thresholded, cv::MORPH_CLOSE,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

  // 二値化マスクで検出領域のみを元画像から抽出
  cv::Mat img_annotated;
  cv_img->image.copyTo(img_annotated, img_thresholded);

  // 二値化画像から輪郭を抽出
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(img_thresholded, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  if (contours.size()) {
    // 検出した全輪郭から最も面積の大きい領域を特定
    std::vector<cv::Moments> object_moments;
    int max_area_i = -1;
    int i = 0;
    for (const auto & contour : contours) {
      object_moments.push_back(cv::moments(contour));
      if (object_moments[max_area_i].m00 < object_moments[i].m00) {
        max_area_i = i;
      }
      i++;
    }

    // 最大領域が最小サイズ閾値を超えていれば物体として認識
    if (object_moments[max_area_i].m00 > MIN_OBJECT_SIZE) {
      // 画像モーメントから物体の重心座標を計算（画像左上が原点）
      cv::Point2d object_point;
      object_point.x = object_moments[max_area_i].m10 / object_moments[max_area_i].m00;
      object_point.y = object_moments[max_area_i].m01 / object_moments[max_area_i].m00;

      RCLCPP_DEBUG_STREAM(this->get_logger(), "Detect at" << object_point << ".");

      // 検出結果を画像上に描画（輪郭と重心）
      const cv::Scalar ANNOTATE_COLOR(256, 0, 256);
      const int ANNOTATE_THICKNESS = 4;
      const int ANNOTATE_RADIUS = 10;
      cv::drawContours(img_annotated, contours, max_area_i, ANNOTATE_COLOR, ANNOTATE_THICKNESS);
      cv::circle(img_annotated, object_point, ANNOTATE_RADIUS, ANNOTATE_COLOR, -1);

      // 画像中心を原点とした座標系に変換
      cv::Point2d translated_object_point;
      translated_object_point.x = object_point.x - msg->width / 2.0;
      translated_object_point.y = object_point.y - msg->height / 2.0;

      // -1.0～1.0の範囲に正規化（追従制御で使いやすい形式）
      cv::Point2d normalized_object_point_;
      if (msg->width != 0 && msg->height != 0) {
        normalized_object_point_.x = translated_object_point.x / (msg->width / 2.0);
        normalized_object_point_.y = translated_object_point.y / (msg->height / 2.0);
      }

      // 正規化座標をPointStampedメッセージとして配信
      geometry_msgs::msg::PointStamped object_point_msg;
      object_point_msg.header = msg->header;
      object_point_msg.point.x = normalized_object_point_.x;
      object_point_msg.point.y = normalized_object_point_.y;
      object_point_publisher_->publish(object_point_msg);
    }
  }

  // 検出結果を描画した画像をROS 2メッセージに変換して配信
  sensor_msgs::msg::Image::SharedPtr img_annotated_msg =
    cv_bridge::CvImage(msg->header, msg->encoding, img_annotated).toImageMsg();
  image_annotated_publisher_->publish(*img_annotated_msg);
}

}  // namespace sciurus17_examples

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(sciurus17_examples::ColorDetection2D)
