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

#include "pose_presets.hpp"

namespace pose_presets
{
// Pose型の位置姿勢を作成
geometry_msgs::msg::Pose generate_pose(
  const double x, const double y, const double z,
  const double roll, const double pitch, const double yaw)
{
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  q.setRPY(roll, pitch, yaw);
  target_pose.orientation = tf2::toMsg(q);
  return target_pose;
}

// 右グリッパを下に向ける姿勢を作成
geometry_msgs::msg::Pose right_arm_downward(const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose = generate_pose(
    x, y, z,
    angles::from_degrees(90), angles::from_degrees(0), angles::from_degrees(0));
  return target_pose;
}

// 左グリッパを下に向ける姿勢を作成
geometry_msgs::msg::Pose left_arm_downward(const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose = generate_pose(
    x, y, z,
    angles::from_degrees(-90), angles::from_degrees(0), angles::from_degrees(0));
  return target_pose;
}
}  // namespace pose_presets
