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
#include "rclcpp_action/rclcpp_action.hpp"
#include "angles/angles.h"

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "control_msgs/action/follow_joint_trajectory.hpp"
using std::placeholders::_1;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

class ObjectTracker : public rclcpp::Node
{
public:

  ObjectTracker()
  : Node("head_camera_tracking_2")
  {
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/head_camera/color/image_raw", 10, std::bind(&ObjectTracker::image_callback, this, _1));

    image_thresholded_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("image_thresholded", 10);
  }

  cv::Point2d get_normalized_object_point()
  {
    return normalized_object_point_;
  }

  bool has_object_point()
  {
    return has_object_point_;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_thresholded_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Point2d pixel_err_;
  bool has_object_point_ = false;
  // -1.0 ~ 1.0に正規化された検出位置
  cv::Point2d normalized_object_point_;

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
    has_object_point_ = d_area > 100000;

    if (has_object_point_) {
      // 画像座標系における物体検出位置（2D）
      cv::Point2d object_point;
      object_point.x = d_m10 / d_area;
      object_point.y = d_m01 / d_area;

      RCLCPP_INFO_STREAM(this->get_logger(), "Detect at" << object_point << ".");

      // 画像の中心を原点とした検出位置に変換
      cv::Point2d translated_object_point;
      translated_object_point.x = object_point.x - msg->width / 2.0;
      translated_object_point.y = object_point.y - msg->height / 2.0;

      if (msg->width != 0 && msg->height) {
        normalized_object_point_.x = translated_object_point.x / msg->width / 2.0;
        normalized_object_point_.y = translated_object_point.y / msg->height / 2.0;
      }
    }
  }
};

class NeckControl : public rclcpp::Node
{
public:

  NeckControl()
  : Node("neck_control")
  {
    this->client_ptr_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this,
      "neck_controller/follow_joint_trajectory");

    state_subscription_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/neck_controller/controller_state", 10, std::bind(&NeckControl::state_callback, this, _1));
  }

  void send_goal(double yaw_angle, double pitch_angle, double seconds)
  {
    using namespace std::placeholders;

    if(!this->client_ptr_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names.push_back("neck_yaw_joint");
    goal_msg.trajectory.joint_names.push_back("neck_pitch_joint");

    trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
    trajectory_point_msg.positions.push_back(yaw_angle);
    trajectory_point_msg.positions.push_back(pitch_angle);
    trajectory_point_msg.time_from_start = rclcpp::Duration::from_seconds(seconds);
    goal_msg.trajectory.points.push_back(trajectory_point_msg);

    this->client_ptr_->async_send_goal(goal_msg);
  }

  double get_current_yaw_angle()
  {
    return current_yaw_angle_;
  }

  double get_current_pitch_angle()
  {
    return current_pitch_angle_;
  }

  bool has_state()
  {
    return has_state_;
  }

private:
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client_ptr_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_subscription_;
  control_msgs::msg::JointTrajectoryControllerState::SharedPtr neck_state_;
  double current_yaw_angle_ = 0;
  double current_pitch_angle_ = 0;
  bool has_state_ = false;

  void state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    current_yaw_angle_ = msg->feedback.positions[0];
    current_pitch_angle_ = msg->feedback.positions[1];
    has_state_ = true;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::WallRate loop_rate(100ms);
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto objeckt_tracker_node = std::make_shared<ObjectTracker>();
  auto neck_control_node = std::make_shared<NeckControl>();
  exec.add_node(objeckt_tracker_node);
  exec.add_node(neck_control_node);
  exec.spin_once();

  // 追従を開始する物体位置の閾値
  const double THRESH_X = 0.05;
  const double THRESH_Y= 0.05;

  // 首角度の初期値
  const double INITIAL_YAW_ANGLE = 0;
  const double INITIAL_PITCH_ANGLE = 0;

  // 首可動範囲
  const double MAX_YAW_ANGLE = angles::from_degrees(120);
  const double MIN_YAW_ANGLE = angles::from_degrees(-120);
  const double MAX_PITCH_ANGLE = angles::from_degrees(50);
  const double MIN_PITCH_ANGLE = angles::from_degrees(-70);

  // 首角度初期化時の制御角度
  const double RESET_ANGLE_VEL = angles::from_degrees(3);

  // 物体が検出されなくなってから初期角度に戻り始めるまでの時間
  const std::chrono::nanoseconds DETECTION_TIMEOUT = 1s;

  // 首角度制御量
  // 値が大きいほど追従速度が速くなる
  const double OPERATION_GAIN_X = 0.5;
  const double OPERATION_GAIN_Y = 0.5;

  // 物体検出時の時刻
  auto detection_timestamp = objeckt_tracker_node->get_clock()->now().nanoseconds();

  // 追従動作開始フラグ
  bool look_object = false;

  // 現在の首角度を取得
  while (!neck_control_node->has_state()) {
    exec.spin_once();
  }
  auto yaw_angle = neck_control_node->get_current_yaw_angle();
  auto pitch_angle = neck_control_node->get_current_pitch_angle();

  while (rclcpp::ok()) {
    // 現在時刻
    auto now = objeckt_tracker_node->get_clock()->now().nanoseconds();

    // 物体を検出したとき追従動作開始フラグをtrueにする
    if (objeckt_tracker_node->has_object_point()) {
      detection_timestamp = now;
      look_object = true;
    } else {
      // 物体を検出しなくなってから1秒後に初期角度へ戻る
      auto lost_time = now - detection_timestamp;
      if (lost_time > DETECTION_TIMEOUT.count()) {
        look_object = false;
      }
    }

    // 物体が検出されたら追従を行う
    if (look_object) {
      // 物体検出位置を取得
      auto object_point = objeckt_tracker_node->get_normalized_object_point();

      // 追従動作のための首角度を計算
      if (std::abs(object_point.x) > THRESH_X) {
        yaw_angle -= object_point.x * OPERATION_GAIN_X;
      }
      if (std::abs(object_point.y) > THRESH_Y) {
        pitch_angle -= object_point.y * OPERATION_GAIN_Y;
      }
    } else {
      // ゆっくりと初期角度へ戻る
      auto diff_yaw_angle = INITIAL_YAW_ANGLE - yaw_angle;
      if (std::abs(diff_yaw_angle) > RESET_ANGLE_VEL) {
        yaw_angle += std::copysign(RESET_ANGLE_VEL, diff_yaw_angle);
      } else {
        yaw_angle = INITIAL_YAW_ANGLE;
      }

      auto diff_pitch_angle = INITIAL_PITCH_ANGLE - pitch_angle;
      if (std::abs(diff_pitch_angle) > RESET_ANGLE_VEL) {
        pitch_angle += std::copysign(RESET_ANGLE_VEL, diff_pitch_angle);
      } else {
        pitch_angle = INITIAL_PITCH_ANGLE;
      }
    }

    // 目標首角度を制限角度内に収める
    yaw_angle = std::clamp(yaw_angle, MIN_YAW_ANGLE, MAX_YAW_ANGLE);
    pitch_angle = std::clamp(pitch_angle, MIN_PITCH_ANGLE, MAX_PITCH_ANGLE);

    // 目標角度に首を動かす
    neck_control_node->send_goal(yaw_angle, pitch_angle, 0.1);

    exec.spin_once();
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
