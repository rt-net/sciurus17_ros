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

#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
using std::placeholders::_1;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class HeadCameraTracking : public rclcpp::Node
{
public:
  HeadCameraTracking(
    rclcpp::Node::SharedPtr move_group_neck_node)
  : Node("head_camera_tracking")
  {
    move_group_neck_ = std::make_shared<MoveGroupInterface>(move_group_neck_node, "neck_group");
    move_group_neck_->setMaxVelocityScalingFactor(0.5);
    move_group_neck_->setMaxAccelerationScalingFactor(0.5);

    move_group_neck_->setNamedTarget("neck_init_pose");
    move_group_neck_->move();

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/head_camera/color/image_raw", 10, std::bind(&HeadCameraTracking::image_callback, this, _1));

    image_thresholded_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("image_thresholded", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_thresholded_publisher_;
  std::shared_ptr<MoveGroupInterface> move_group_neck_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // オレンジ色の物体を検出するようにHSVの範囲を設定
    // 周囲の明るさ等の動作環境に合わせて調整
    const int LOW_H = 5, HIGH_H = 20;
    const int LOW_S = 120, HIGH_S = 255;
    const int LOW_V = 120, HIGH_V = 255;

    // ウェブカメラの画像を受け取る
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

    // 画像の検出領域におけるモーメントを計算
    cv::Moments moment = moments(img_thresholded);
    double d_m01 = moment.m01;
    double d_m10 = moment.m10;
    double d_area = moment.m00;

    // 検出した領域のピクセル数が10000より大きい場合
    if (d_area > 10000) {
      // 画像座標系における把持対象物の位置（2D）
      const double pixel_x = d_m10 / d_area;
      const double pixel_y = d_m01 / d_area;
      const double pixel_err_x = pixel_x - msg->width / 2.0;
      const double pixel_err_y = pixel_y - msg->height / 2.0;
      const cv::Point2d point(pixel_err_x, pixel_err_y);
      RCLCPP_INFO_STREAM(this->get_logger(), "Detect at" << point << ".");

      // 現在角度をベースに、目標角度を作成する
      auto joint_values = move_group_neck_->getCurrentJointValues();

      const double OPERATION_GAIN_X = 0.001;
      // 首を左に向ける
      joint_values[0] = joint_values[0] - pixel_err_x * OPERATION_GAIN_X;
      move_group_neck_->setJointValueTarget(joint_values);
      move_group_neck_->move();

      // 閾値による二値化画像を配信
      sensor_msgs::msg::Image::SharedPtr img_thresholded_msg =
        cv_bridge::CvImage(msg->header, "mono8", img_thresholded).toImageMsg();
      image_thresholded_publisher_->publish(*img_thresholded_msg);
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_neck_node = rclcpp::Node::make_shared("move_group_neck_node", node_options);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto head_camera_tracking_node = std::make_shared<HeadCameraTracking>(
    move_group_neck_node);
  exec.add_node(head_camera_tracking_node);
  exec.add_node(move_group_neck_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
