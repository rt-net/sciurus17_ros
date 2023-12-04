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

class HeadCameraTracking : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;

  HeadCameraTracking(
  )
  : Node("head_camera_tracking")
  {
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/head_camera/color/image_raw", 10, std::bind(&HeadCameraTracking::image_callback, this, _1));

    image_thresholded_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("image_thresholded", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(HeadCameraTracking::tracking_callback, this));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_thresholded_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr color_image_;

  bool color_detection(cv::Point2d& pixel_err)
  {
    // オレンジ色の物体を検出するようにHSVの範囲を設定
    // 周囲の明るさ等の動作環境に合わせて調整
    const int LOW_H = 5, HIGH_H = 20;
    const int LOW_S = 120, HIGH_S = 255;
    const int LOW_V = 120, HIGH_V = 255;

    // ウェブカメラの画像を受け取る
    auto cv_img = cv_bridge::toCvShare(color_image_, color_image_->encoding);

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
    bool object_detected = d_area > 10000;

    if (object_detected) {
      // 画像座標系における把持対象物の位置（2D）
      const double pixel_x = d_m10 / d_area;
      const double pixel_y = d_m01 / d_area;
      const double pixel_err_x = pixel_x - color_image_->width / 2.0;
      const double pixel_err_y = pixel_y - color_image_->height / 2.0;
      pixel_err = cv::Point2d(pixel_err_x, pixel_err_y);
      RCLCPP_INFO_STREAM(this->get_logger(), "Detect at" << pixel_err << ".");

      // 閾値による二値化画像を配信
      sensor_msgs::msg::Image::SharedPtr img_thresholded_msg =
        cv_bridge::CvImage(color_image_->header, "mono8", img_thresholded).toImageMsg();
      image_thresholded_publisher_->publish(*img_thresholded_msg);
    }
    return object_detected;
  }

  void tracking_callback()
  {
    auto neck = NeckControl();
    cv::Point2d pixel_err;
    if(color_detection(pixel_err)) {
      const double OPERATION_GAIN_X = 0.001;
      auto yaw_angle = current_yaw_angle_ - pixel_err_x * OPERATION_GAIN_X;
      neck.send_goal(yaw_angle, 0, 0.01);
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    color_image_ = msg;
  }
};

class NeckControl : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;

  NeckControl(
  )
  : Node("neck_control")
  {
    this->client_ptr_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this,
      "neck_controller/follow_joint_trajectory");

    state_subscription_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/neck_controller/state", 10, std::bind(&NeckControl::state_callback, this, _1));
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

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.feedback_callback =
      std::bind(&NeckControl::feedback_callback, this, _1, _2);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client_ptr_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_subscription_;
  double current_yaw_angle_;
  double current_pitch_angle_;

  void state_callback(const control_msgs::msg::JointTrajectoryControllerState)
  void feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
  {
    current_yaw_angle_ = feedback->actual.positions[0];
    current_pitch_angle_ = feedback->actual.positions[1];
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto head_camera_tracking_node = std::make_shared<HeadCameraTracking>();
  auto neck_control_node = std::make_shared<NeckControl>();
  exec.add_node(head_camera_tracking_node);
  exec.add_node(neck_control_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
