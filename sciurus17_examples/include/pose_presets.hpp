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

#ifndef POSE_PRESETS_HPP_
#define POSE_PRESETS_HPP_

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pose_presets
{
// Pose型の位置姿勢を作成
geometry_msgs::msg::Pose generate_pose(
  const double x, const double y, const double z,
  const double roll, const double pitch, const double yaw);
// 右グリッパを下に向ける姿勢を作成
geometry_msgs::msg::Pose right_arm_downward(const double x, const double y, const double z);
// 左グリッパを下に向ける姿勢を作成
geometry_msgs::msg::Pose left_arm_downward(const double x, const double y, const double z);
}  // namespace pose_presets

#endif  // POSE_PRESETS_HPP_
