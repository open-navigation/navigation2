// Copyright (c) 2022 Adam Aposhian
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

#include <memory>

#include "nav2_util/path_utils.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_util
{

void append_transform_to_path(
  nav_msgs::msg::Path & path,
  tf2::Transform & transform)
{
  auto & last_pose = path.poses.back();
  geometry_msgs::msg::TransformStamped transform_msg(tf2::toMsg(
      tf2::Stamped<tf2::Transform>(
        transform, tf2::getTimestamp(last_pose), tf2::getFrameId(last_pose))));
  path.poses.emplace_back();
  auto & pose_stamped = path.poses.back();
  tf2::doTransform(
    last_pose,
    pose_stamped,
    transform_msg
  );
}

void Straight::append(nav_msgs::msg::Path & path, double spacing) const
{
  auto num_points = std::floor(length_ / spacing);
  path.poses.reserve(path.poses.size() + num_points);
  tf2::Transform translation;
  translation.setOrigin(tf2::Vector3(spacing, 0.0, 0.0));
  for (size_t i = 1; i <= num_points; ++i) {
    append_transform_to_path(path, translation);
  }
}

void Arc::append(nav_msgs::msg::Path & path, double spacing) const
{
  double length = radius_ * radians_;
  size_t num_points = std::floor(length / spacing);
  double radians_per_step = radians_ / num_points;
  tf2::Transform transform(
    tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), radians_per_step),
    tf2::Vector3(radius_ * cos(radians_per_step), radius_ * sin(radians_per_step), 0.0));
  path.poses.reserve(path.poses.size() + num_points);
  for (size_t i = 0; i < num_points; ++i) {
    append_transform_to_path(path, transform);
  }
}

nav_msgs::msg::Path generate_path(
  geometry_msgs::msg::PoseStamped start,
  std::initializer_list<std::unique_ptr<PathSegment>> segments,
  double spacing)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(start);
  for (const auto & segment : segments) {
    segment->append(path, spacing);
  }
  return path;
}

}  // namespace nav2_util
