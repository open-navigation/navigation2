// Copyright (c) 2022 Samsung R&D Institute Russia
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

#include "nav2_collision_monitor/collision_detector_node.hpp"

#include <exception>
#include <utility>
#include <functional>

#include "tf2_ros/create_timer_ros.h"

#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace nav2_collision_monitor
{

CollisionDetector::CollisionDetector(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("collision_detector", "", options)
{
}

CollisionDetector::~CollisionDetector()
{
  polygons_.clear();
  sources_.clear();
}

nav2_util::CallbackReturn
CollisionDetector::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  state_pub_ = this->create_publisher<nav2_msgs::msg::CollisionDetectorState>(
    "collision_detector_state", rclcpp::SystemDefaultsQoS());

  collision_points_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/collision_points_marker", 1);

  // Obtaining ROS parameters
  if (!getParameters()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionDetector::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publisher
  state_pub_->on_activate();
  collision_points_marker_pub_->on_activate();

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->activate();
  }

  // Creating timer
  timer_ = this->create_timer(
    std::chrono::duration<double>{1.0 / frequency_},
    std::bind(&CollisionDetector::process, this));

  // Creating bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionDetector::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Resetting timer
  timer_.reset();

  // Deactivating lifecycle publishers
  state_pub_->on_deactivate();
  collision_points_marker_pub_->on_deactivate();

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->deactivate();
  }

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionDetector::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  state_pub_.reset();
  collision_points_marker_pub_.reset();

  polygons_.clear();
  sources_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionDetector::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool CollisionDetector::getParameters()
{
  std::string base_frame_id, odom_frame_id;
  tf2::Duration transform_tolerance;
  rclcpp::Duration source_timeout(2.0, 0.0);

  auto node = shared_from_this();

  nav2_util::declare_parameter_if_not_declared(
    node, "frequency", rclcpp::ParameterValue(10.0));
  frequency_ = get_parameter("frequency").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, "base_frame_id", rclcpp::ParameterValue("base_footprint"));
  base_frame_id = get_parameter("base_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "odom_frame_id", rclcpp::ParameterValue("odom"));
  odom_frame_id = get_parameter("odom_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  transform_tolerance =
    tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "source_timeout", rclcpp::ParameterValue(2.0));
  source_timeout =
    rclcpp::Duration::from_seconds(get_parameter("source_timeout").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "base_shift_correction", rclcpp::ParameterValue(true));
  const bool base_shift_correction =
    get_parameter("base_shift_correction").as_bool();

  if (!configurePolygons(base_frame_id, transform_tolerance)) {
    return false;
  }

  if (!configureSources(
      base_frame_id, odom_frame_id, transform_tolerance, source_timeout,
      base_shift_correction))
  {
    return false;
  }

  return true;
}

bool CollisionDetector::configurePolygons(
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
{
  try {
    auto node = shared_from_this();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, "polygons", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> polygon_names = get_parameter("polygons").as_string_array();
    for (std::string polygon_name : polygon_names) {
      // Leave it not initialized: the will cause an error if it will not set
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name + ".type", rclcpp::PARAMETER_STRING);
      const std::string polygon_type = get_parameter(polygon_name + ".type").as_string();

      if (polygon_type == "polygon") {
        polygons_.push_back(
          std::make_shared<Polygon>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else if (polygon_type == "circle") {
        polygons_.push_back(
          std::make_shared<Circle>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else if (polygon_type == "velocity_polygon") {
        polygons_.push_back(
          std::make_shared<VelocityPolygon>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown polygon type: %s",
          polygon_name.c_str(), polygon_type.c_str());
        return false;
      }

      // Configure last added polygon
      if (!polygons_.back()->configure()) {
        return false;
      }

      // warn if the added polygon's action_type_ is not different than "none"
      auto action_type = polygons_.back()->getActionType();
      if (action_type != DO_NOTHING) {
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: The action_type of the polygon is different than \"none\" which is "
          "not supported in the collision detector.",
          polygon_name.c_str());
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionDetector::configureSources(
  const std::string & base_frame_id,
  const std::string & odom_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
{
  try {
    auto node = shared_from_this();

    // Leave it to be not initialized to intentionally cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> source_names = get_parameter("observation_sources").as_string_array();
    for (std::string source_name : source_names) {
      nav2_util::declare_parameter_if_not_declared(
        node, source_name + ".type",
        rclcpp::ParameterValue("scan"));  // Laser scanner by default
      const std::string source_type = get_parameter(source_name + ".type").as_string();

      if (source_type == "scan") {
        std::shared_ptr<Scan> s = std::make_shared<Scan>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        s->configure();

        sources_.push_back(s);
      } else if (source_type == "pointcloud") {
        std::shared_ptr<PointCloud> p = std::make_shared<PointCloud>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        p->configure();

        sources_.push_back(p);
      } else if (source_type == "range") {
        std::shared_ptr<Range> r = std::make_shared<Range>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        r->configure();

        sources_.push_back(r);
      } else if (source_type == "polygon") {
        std::shared_ptr<PolygonSource> ps = std::make_shared<PolygonSource>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);
        ps->configure();

        sources_.push_back(ps);
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown source type: %s",
          source_name.c_str(), source_type.c_str());
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

void CollisionDetector::process()
{
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->now();

  // Points array collected from different data sources in a robot base frame
  std::vector<Point> collision_points;

  std::unique_ptr<nav2_msgs::msg::CollisionDetectorState> state_msg =
    std::make_unique<nav2_msgs::msg::CollisionDetectorState>();

  // Fill collision_points array from different data sources
  for (std::shared_ptr<Source> source : sources_) {
    if (source->getEnabled()) {
      if (!source->getData(curr_time, collision_points) &&
        source->getSourceTimeout().seconds() != 0.0)
      {
        RCLCPP_WARN(
          get_logger(),
          "Invalid source %s detected."
          " Either due to data not published yet, or to lack of new data received within the"
          " sensor timeout, or if impossible to transform data to base frame",
          source->getSourceName().c_str());
      }
    }
  }

  if (collision_points_marker_pub_->get_subscription_count() > 0) {
    // visualize collision points with markers
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = get_parameter("base_frame_id").as_string();
    marker.header.stamp = rclcpp::Time(0, 0);
    marker.ns = "collision_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.frame_locked = true;

    for (const auto & point : collision_points) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }
    marker_array->markers.push_back(marker);
    collision_points_marker_pub_->publish(std::move(marker_array));
  }

  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (!polygon->getEnabled()) {
      continue;
    }
    state_msg->polygons.push_back(polygon->getName());
    polygon->newCollisionPoints(collision_points, curr_time);

    state_msg->detections.push_back(polygon->shouldApplyAction());
  }

  state_pub_->publish(std::move(state_msg));

  // Publish polygons for better visualization
  publishPolygons();
}

void CollisionDetector::publishPolygons() const
{
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (polygon->getEnabled()) {
      polygon->publish();
    }
  }
}

}  // namespace nav2_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionDetector)
