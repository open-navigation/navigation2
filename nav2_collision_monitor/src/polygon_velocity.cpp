// Copyright (c) 2022 Dexory
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

#include "nav2_collision_monitor/polygon_velocity.hpp"
#include "nav2_collision_monitor/polygon.hpp"

#include "nav2_util/node_utils.hpp"


namespace nav2_collision_monitor
{

PolygonVelocity::PolygonVelocity(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::string & polygon_velocity_name)
: node_(node), polygon_name_(polygon_name), polygon_velocity_name_(polygon_velocity_name)
{
  RCLCPP_INFO(logger_, "[%s]: Creating PolygonVelocity", polygon_velocity_name_.c_str());
}


PolygonVelocity::~PolygonVelocity()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying PolygonVelocity", polygon_velocity_name_.c_str());
  poly_.clear();
}


bool PolygonVelocity::getParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // do some try catch?
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + polygon_velocity_name_ + ".points",
    rclcpp::PARAMETER_DOUBLE_ARRAY);
  std::vector<double> polygon_points = node->get_parameter(
    polygon_name_ + "." + polygon_velocity_name_ + ".points").as_double_array();

  if (!Polygon::setPolygonShape(polygon_points, poly_)) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Polygon has incorrect points description",
      polygon_name_.c_str());
    return false;
  }

  // holonomic param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + polygon_velocity_name_ + ".holonomic",
    rclcpp::ParameterValue(false));
  holonomic_ =
    node->get_parameter(
    polygon_name_ + "." + polygon_velocity_name_ +
    ".holonomic").as_bool();

  // linear_max param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + polygon_velocity_name_ + ".linear_max",
    rclcpp::ParameterValue(0.0));
  linear_max_ =
    node->get_parameter(
    polygon_name_ + "." + polygon_velocity_name_ +
    ".linear_max").as_double();

  // linear_min param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + polygon_velocity_name_ + ".linear_min",
    rclcpp::ParameterValue(0.0));
  linear_min_ =
    node->get_parameter(
    polygon_name_ + "." + polygon_velocity_name_ +
    ".linear_min").as_double();

  // direction_end_angle param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + polygon_velocity_name_ + ".direction_end_angle",
    rclcpp::ParameterValue(0.0));
  direction_end_angle_ =
    node->get_parameter(
    polygon_name_ + "." + polygon_velocity_name_ +
    ".direction_end_angle").as_double();

  // direction_start_angle param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + polygon_velocity_name_ + ".direction_start_angle",
    rclcpp::ParameterValue(0.0));
  direction_start_angle_ =
    node->get_parameter(
    polygon_name_ + "." + polygon_velocity_name_ +
    ".direction_start_angle").as_double();

  // theta_max param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + polygon_velocity_name_ + ".theta_max", rclcpp::ParameterValue(
      0.0));
  theta_max_ =
    node->get_parameter(polygon_name_ + "." + polygon_velocity_name_ + ".theta_max").as_double();

  // theta_min param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + polygon_velocity_name_ + ".theta_min", rclcpp::ParameterValue(
      0.0));
  theta_min_ =
    node->get_parameter(polygon_name_ + "." + polygon_velocity_name_ + ".theta_min").as_double();

  return true;
}

bool PolygonVelocity::isInRange(const Velocity & cmd_vel_in)
{
  if (holonomic_) {
    const double twist_linear = std::hypot(cmd_vel_in.x, cmd_vel_in.y);

    // check if direction in angle range(min -> max)
    double direction = std::atan2(cmd_vel_in.y, cmd_vel_in.x);
    bool direction_in_range;
    if (direction_start_angle_ <= direction_end_angle_) {
      direction_in_range =
        (direction >= direction_start_angle_ && direction <= direction_end_angle_);
    } else {
      direction_in_range =
        (direction >= direction_start_angle_ || direction <= direction_end_angle_);
    }

    return twist_linear <= linear_max_ &&
           twist_linear >= linear_min_ &&
           direction_in_range &&
           cmd_vel_in.tw <= theta_max_ &&
           cmd_vel_in.tw >= theta_min_;
  } else {
    // non-holonomic
    return cmd_vel_in.x <= linear_max_ &&
           cmd_vel_in.x >= linear_min_ &&
           cmd_vel_in.tw <= theta_max_ &&
           cmd_vel_in.tw >= theta_min_;
  }
}

std::vector<Point> PolygonVelocity::getPolygon()
{
  return poly_;
}


}  // namespace nav2_collision_monitor
