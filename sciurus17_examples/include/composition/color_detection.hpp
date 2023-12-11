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

#ifndef COMPOSITION__COLOR_DETECTION_HPP_
#define COMPOSITION__COLOR_DETECTION_HPP_

#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace sciurus17_examples
{

class ColorDetection : public rclcpp::Node
{
public:
  explicit ColorDetection(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_thresholded_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr object_point_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Point2d pixel_err_;
  // -1.0 ~ 1.0に正規化された検出位置
  cv::Point2d normalized_object_point_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

}  // namespace sciurus17_examples

#endif // COMPOSITION__COLOR_DETECTION_HPP_