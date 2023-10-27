// Copyright (c) 2023 Pixel Robotics GmbH
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

#include "nav2_collision_monitor/polygon_source.hpp"

#include <functional>
#include <cmath>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "tf2/transform_datatypes.h"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace nav2_collision_monitor
{

PolygonSource::PolygonSource(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout, base_shift_correction)
{
}

PolygonSource::~PolygonSource()
{
  data_sub_.reset();
}

void PolygonSource::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  getParameters(source_topic);

  rclcpp::QoS qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    source_topic, qos,
    std::bind(&PolygonSource::dataCallback, this, std::placeholders::_1));
}

void PolygonSource::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data) const
{
  // Ignore data from the source if it is not being published yet or
  // not published for a long time
  if (data_.empty()) {
    return;
  }

  tf2::Stamped<tf2::Transform> tf_transform;
  for (const auto & polygon_stamped : data_) {
    if (base_shift_correction_) {
      // Obtaining the transform to get data from source frame and time where it was received
      // to the base frame and current time
      if (
        !nav2_util::getTransform(
          polygon_stamped.header.frame_id, polygon_stamped.header.stamp,
          base_frame_id_, curr_time, global_frame_id_,
          transform_tolerance_, tf_buffer_, tf_transform))
      {
        return;
      }
    } else {
      // Obtaining the transform to get data from source frame to base frame without time shift
      // considered. Less accurate but much more faster option not dependent on state estimation
      // frames.
      if (
        !nav2_util::getTransform(
          polygon_stamped.header.frame_id, base_frame_id_,
          transform_tolerance_, tf_buffer_, tf_transform))
      {
        return;
      }
    }
    geometry_msgs::msg::PolygonStamped poly_out;
    geometry_msgs::msg::TransformStamped tf = tf2::toMsg(tf_transform);
    tf2::doTransform(polygon_stamped, poly_out, tf);
    convertPolygonStampedToPoints(poly_out, data);
  }
}

void PolygonSource::convertPolygonStampedToPoints(
  const geometry_msgs::msg::PolygonStamped & polygon, std::vector<Point> & data) const
{
  // Iterate over the vertices of the polygon
  for (size_t i = 0; i < polygon.polygon.points.size(); ++i) {
    const auto & current_point = polygon.polygon.points[i];
    const auto & next_point = polygon.polygon.points[(i + 1) % polygon.polygon.points.size()];

    // Calculate the distance between the current and next points
    double segment_length =
      std::hypot(next_point.x - current_point.x, next_point.y - current_point.y);

    // Calculate the number of points to sample in the current segment
    size_t num_points_in_segment =
      std::max(static_cast<size_t>(segment_length / sampling_distance_), static_cast<size_t>(1));

    // Calculate the step size for each pair of vertices
    const double dx = (next_point.x - current_point.x) / num_points_in_segment;
    const double dy = (next_point.y - current_point.y) / num_points_in_segment;

    // Sample the points with equal spacing
    for (size_t j = 0; j <= num_points_in_segment; ++j) {
      Point p;
      p.x = current_point.x + j * dx;
      p.y = current_point.y + j * dy;
      data.push_back(p);
    }
  }
}

void PolygonSource::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".sampling_distance", rclcpp::ParameterValue(0.1));
  sampling_distance_ = node->get_parameter(source_name_ + ".sampling_distance").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".polygon_similarity_threshold", rclcpp::ParameterValue(2.0));
  polygon_similarity_threshold_ = node->get_parameter(source_name_ + ".polygon_similarity_threshold").as_double();
}

void PolygonSource::dataCallback(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  auto curr_time = node->now();

  // remove old data
  data_.erase(
    std::remove_if(
      data_.begin(), data_.end(),
      [this, curr_time](const geometry_msgs::msg::PolygonStamped & polygon_stamped) {
        return curr_time - rclcpp::Time(polygon_stamped.header.stamp) > source_timeout_;
      }), data_.end());

  // check if older similar polygon exists already and replace it with the new one
  for (auto & polygon_stamped : data_) {
    if (rclcpp::Time(msg->header.stamp) > rclcpp::Time(polygon_stamped.header.stamp) &&
      arePolygonsSimilar(msg->polygon, polygon_stamped.polygon))
    {
      polygon_stamped = *msg;
      return;
    }
  }
  data_.push_back(*msg);
}

bool PolygonSource::arePolygonsSimilar(
  const geometry_msgs::msg::Polygon & polygon1,
  const geometry_msgs::msg::Polygon & polygon2)
{
  // Compare the number of points in the polygons
  if (polygon1.points.size() != polygon2.points.size()) {
    return false;
  }

  // Compare the distance between each pair of corresponding points
  for (size_t i = 0; i < polygon1.points.size(); ++i) {
    double dx = polygon1.points[i].x - polygon2.points[i].x;
    double dy = polygon1.points[i].y - polygon2.points[i].y;
    double dz = polygon1.points[i].z - polygon2.points[i].z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance > polygon_similarity_threshold_) {
      return false;
    }
  }

  return true;
}

}  // namespace nav2_collision_monitor
