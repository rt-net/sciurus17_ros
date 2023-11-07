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
#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pose_presets
{
geometry_msgs::msg::Pose PosePresets::generate_pose(
  double x, double y, double z, double roll, double pitch, double yaw)
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

geometry_msgs::msg::Pose PosePresets::right_downward(double x, double y, double z)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose = generate_pose(x, y, z,
    angles::from_degrees(90), angles::from_degrees(0), angles::from_degrees(0));
  return target_pose;
}

geometry_msgs::msg::Pose PosePresets::left_downward(double x, double y, double z)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose = generate_pose(x, y, z,
    angles::from_degrees(-90), angles::from_degrees(0), angles::from_degrees(0));
  return target_pose;
}
}  // namespace pose_presets
