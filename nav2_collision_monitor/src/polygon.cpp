// Copyright (c) 2022 Samsung R&D Institute Russia //
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

#include "nav2_collision_monitor/polygon.hpp"

#include <exception>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "tf2/transform_datatypes.h"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/array_parser.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

namespace nav2_collision_monitor
{

Polygon::Polygon(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: node_(node), polygon_name_(polygon_name), action_type_(DO_NOTHING),
  slowdown_ratio_(0.0), linear_limit_(0.0), angular_limit_(0.0),
  footprint_sub_(nullptr), tf_buffer_(tf_buffer),
  base_frame_id_(base_frame_id), transform_tolerance_(transform_tolerance),
  should_apply_action_(false), last_time_data_in_(rclcpp::Time(0,0)), 
  last_time_data_out_(rclcpp::Time(0,0))
{
  RCLCPP_INFO(logger_, "[%s]: Creating Polygon", polygon_name_.c_str());
}

Polygon::~Polygon()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Polygon", polygon_name_.c_str());
  polygon_sub_.reset();
  polygon_pub_.reset();
  poly_.clear();
  dyn_params_handler_.reset();
}

bool Polygon::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string polygon_sub_topic, polygon_pub_topic, footprint_topic;

  if (!getParameters(polygon_sub_topic, polygon_pub_topic, footprint_topic)) {
    return false;
  }

  createSubscription(polygon_sub_topic);

  if (!footprint_topic.empty()) {
    RCLCPP_INFO(
      logger_,
      "[%s]: Making footprint subscriber on %s topic",
      polygon_name_.c_str(), footprint_topic.c_str());
    footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
      node, footprint_topic, *tf_buffer_,
      base_frame_id_, tf2::durationToSec(transform_tolerance_));
  }

  if (visualize_) {
    // Fill polygon_ for future usage
    polygon_.header.frame_id = base_frame_id_;
    std::vector<Point> poly;
    getPolygon(poly);
    for (const Point & p : poly) {
      geometry_msgs::msg::Point32 p_s;
      p_s.x = p.x;
      p_s.y = p.y;
      // p_s.z will remain 0.0
      polygon_.polygon.points.push_back(p_s);
    }

    rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();  // set to default
    polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      polygon_pub_topic, polygon_qos);
  }

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Polygon::dynamicParametersCallback, this, std::placeholders::_1));

  return true;
}

void Polygon::activate()
{
  if (visualize_) {
    polygon_pub_->on_activate();
  }
}

void Polygon::deactivate()
{
  if (visualize_) {
    polygon_pub_->on_deactivate();
  }
}

std::string Polygon::getName() const
{
  return polygon_name_;
}

ActionType Polygon::getActionType() const
{
  return action_type_;
}

bool Polygon::getEnabled() const
{
  return enabled_;
}

int Polygon::getMinPoints() const
{
  return min_points_;
}

double Polygon::getSlowdownRatio() const
{
  return slowdown_ratio_;
}

double Polygon::getLinearLimit() const
{
  return linear_limit_;
}

double Polygon::getAngularLimit() const
{
  return angular_limit_;
}

double Polygon::getTimeBeforeCollision() const
{
  return time_before_collision_;
}

void Polygon::getPolygon(std::vector<Point> & poly) const
{
  poly = poly_;
}

bool Polygon::isShapeSet()
{
  if (poly_.empty()) {
    RCLCPP_WARN(logger_, "[%s]: Polygon shape is not set yet", polygon_name_.c_str());
    return false;
  }
  return true;
}

void Polygon::updatePolygon(const Velocity & /*cmd_vel_in*/)
{
  if (footprint_sub_ != nullptr) {
    // Get latest robot footprint from footprint subscriber
    std::vector<geometry_msgs::msg::Point> footprint_vec;
    std_msgs::msg::Header footprint_header;
    footprint_sub_->getFootprintInRobotFrame(footprint_vec, footprint_header);

    std::size_t new_size = footprint_vec.size();
    poly_.resize(new_size);
    polygon_.header.frame_id = base_frame_id_;
    polygon_.polygon.points.resize(new_size);

    geometry_msgs::msg::Point32 p_s;
    for (std::size_t i = 0; i < new_size; i++) {
      poly_[i] = {footprint_vec[i].x, footprint_vec[i].y};
      p_s.x = footprint_vec[i].x;
      p_s.y = footprint_vec[i].y;
      polygon_.polygon.points[i] = p_s;
    }
  } else if (!polygon_.header.frame_id.empty() && polygon_.header.frame_id != base_frame_id_) {
    // Polygon is published in another frame: correct poly_ vertices to the latest frame state
    std::size_t new_size = polygon_.polygon.points.size();

    // Get the transform from PolygonStamped frame to base_frame_id_
    tf2::Stamped<tf2::Transform> tf_transform;
    if (
      !nav2_util::getTransform(
        polygon_.header.frame_id, base_frame_id_,
        transform_tolerance_, tf_buffer_, tf_transform))
    {
      return;
    }

    // Correct main poly_ vertices
    poly_.resize(new_size);
    for (std::size_t i = 0; i < new_size; i++) {
      // Transform point coordinates from PolygonStamped frame -> to base frame
      tf2::Vector3 p_v3_s(polygon_.polygon.points[i].x, polygon_.polygon.points[i].y, 0.0);
      tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

      // Fill poly_ array
      poly_[i] = {p_v3_b.x(), p_v3_b.y()};
    }
  }
}

int Polygon::getPointsInside(const std::vector<Point> & points) const
{
  int num = 0;
  for (const Point & point : points) {
    if (isPointInside(point)) {
      num++;
    }
  }
  return num;
}

bool Polygon::shouldApplyAction() const
{
    return should_apply_action_;
}

void Polygon::newCollisionPoints(
  const std::vector<Point> & collision_points,
  const rclcpp::Time & current_time) 
{
  bool is_in = getPointsInside(collision_points) >= getMinPoints();
  // Record time of last data in the same direction as the current decision
  if (should_apply_action_ && is_in) {
    last_time_data_in_ = current_time;
  } else if (!should_apply_action_ && !is_in){
    last_time_data_out_ = current_time;
  }

  // Update the decision when required
  if (should_apply_action_ 
    && (last_time_data_out_ - current_time).seconds() >= min_time_out_sec_ ) {
      should_apply_action_ = false;
  }
  else if (!should_apply_action_ 
    && (last_time_data_in_ - current_time).seconds() >= min_time_out_sec_) {
      should_apply_action_ = true;
  }
}

double Polygon::getCollisionTime(
  const std::vector<Point> & collision_points,
  const Velocity & velocity) const
{
  // Initial robot pose is {0,0} in base_footprint coordinates
  Pose pose = {0.0, 0.0, 0.0};
  Velocity vel = velocity;

  // Array of points transformed to the frame concerned with pose on each simulation step
  std::vector<Point> points_transformed = collision_points;

  // Check static polygon
  if (getPointsInside(points_transformed) >= min_points_) {
    return 0.0;
  }

  // Robot movement simulation
  for (double time = 0.0; time <= time_before_collision_; time += simulation_time_step_) {
    // Shift the robot pose towards to the vel during simulation_time_step_ time interval
    // NOTE: vel is changing during the simulation
    projectState(simulation_time_step_, pose, vel);
    // Transform collision_points to the frame concerned with current robot pose
    points_transformed = collision_points;
    transformPoints(pose, points_transformed);
    // If the collision occurred on this stage, return the actual time before a collision
    // as if robot was moved with given velocity
    if (getPointsInside(points_transformed) >= min_points_) {
      return time;
    }
  }

  // There is no collision
  return -1.0;
}

void Polygon::publish()
{
  if (!visualize_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Actualize the time to current and publish the polygon
  polygon_.header.stamp = node->now();
  auto msg = std::make_unique<geometry_msgs::msg::PolygonStamped>(polygon_);
  polygon_pub_->publish(std::move(msg));
}

bool Polygon::getCommonParameters(
  std::string & polygon_sub_topic,
  std::string & polygon_pub_topic,
  std::string & footprint_topic,
  bool use_dynamic_sub_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {
    // Get action type.
    // Leave it not initialized: the will cause an error if it will not set.
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".action_type", rclcpp::PARAMETER_STRING);
    const std::string at_str =
      node->get_parameter(polygon_name_ + ".action_type").as_string();
    if (at_str == "stop") {
      action_type_ = STOP;
    } else if (at_str == "slowdown") {
      action_type_ = SLOWDOWN;
    } else if (at_str == "limit") {
      action_type_ = LIMIT;
    } else if (at_str == "approach") {
      action_type_ = APPROACH;
    } else if (at_str == "none") {
      action_type_ = DO_NOTHING;
    } else {  // Error if something else
      RCLCPP_ERROR(logger_, "[%s]: Unknown action type: %s", polygon_name_.c_str(), at_str.c_str());
      return false;
    }

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".enabled", rclcpp::ParameterValue(true));
    enabled_ = node->get_parameter(polygon_name_ + ".enabled").as_bool();

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".min_points", rclcpp::ParameterValue(4));
    min_points_ = node->get_parameter(polygon_name_ + ".min_points").as_int();

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".min_time_in_sec", rclcpp::ParameterValue(1));
    min_time_in_sec_ = node->get_parameter(polygon_name_ + ".min_time_in_sec").as_double();

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".min_time_out_sec", rclcpp::ParameterValue(1));
    min_time_out_sec_ = node->get_parameter(polygon_name_ + ".min_time_out_sec").as_double();

    try {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".max_points", rclcpp::PARAMETER_INTEGER);
      min_points_ = node->get_parameter(polygon_name_ + ".max_points").as_int() + 1;
      RCLCPP_WARN(
        logger_,
        "[%s]: \"max_points\" parameter was deprecated. Use \"min_points\" instead to specify "
        "the minimum number of data readings within a zone to trigger the action",
        polygon_name_.c_str());
    } catch (const std::exception &) {
      // This is normal situation: max_points parameter should not being declared
    }

    if (action_type_ == SLOWDOWN) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".slowdown_ratio", rclcpp::ParameterValue(0.5));
      slowdown_ratio_ = node->get_parameter(polygon_name_ + ".slowdown_ratio").as_double();
    }

    if (action_type_ == LIMIT) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".linear_limit", rclcpp::ParameterValue(0.5));
      linear_limit_ = node->get_parameter(polygon_name_ + ".linear_limit").as_double();
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".angular_limit", rclcpp::ParameterValue(0.5));
      angular_limit_ = node->get_parameter(polygon_name_ + ".angular_limit").as_double();
    }

    if (action_type_ == APPROACH) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".time_before_collision", rclcpp::ParameterValue(2.0));
      time_before_collision_ =
        node->get_parameter(polygon_name_ + ".time_before_collision").as_double();
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".simulation_time_step", rclcpp::ParameterValue(0.1));
      simulation_time_step_ =
        node->get_parameter(polygon_name_ + ".simulation_time_step").as_double();
    }

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".visualize", rclcpp::ParameterValue(false));
    visualize_ = node->get_parameter(polygon_name_ + ".visualize").as_bool();
    if (visualize_) {
      // Get polygon topic parameter in case if it is going to be published
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".polygon_pub_topic", rclcpp::ParameterValue(polygon_name_));
      polygon_pub_topic = node->get_parameter(polygon_name_ + ".polygon_pub_topic").as_string();
    }

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".polygon_subscribe_transient_local", rclcpp::ParameterValue(false));
    polygon_subscribe_transient_local_ =
      node->get_parameter(polygon_name_ + ".polygon_subscribe_transient_local").as_bool();

    if (use_dynamic_sub_topic) {
      if (action_type_ != APPROACH) {
        // Get polygon sub topic
        nav2_util::declare_parameter_if_not_declared(
          node, polygon_name_ + ".polygon_sub_topic", rclcpp::PARAMETER_STRING);
        polygon_sub_topic =
          node->get_parameter(polygon_name_ + ".polygon_sub_topic").as_string();
      } else {
        // Obtain the footprint topic to make a footprint subscription for approach polygon
        nav2_util::declare_parameter_if_not_declared(
          node, polygon_name_ + ".footprint_topic",
          rclcpp::ParameterValue("local_costmap/published_footprint"));
        footprint_topic =
          node->get_parameter(polygon_name_ + ".footprint_topic").as_string();
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting common polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

bool Polygon::getParameters(
  std::string & polygon_sub_topic,
  std::string & polygon_pub_topic,
  std::string & footprint_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Clear the subscription topics. They will be set later, if necessary.
  polygon_sub_topic.clear();
  footprint_topic.clear();

  bool use_dynamic_sub = true;  // if getting parameter points fails, use dynamic subscription
  try {
    // Leave it uninitialized: it will throw an inner exception if the parameter is not set
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".points", rclcpp::PARAMETER_STRING);
    std::string poly_string =
      node->get_parameter(polygon_name_ + ".points").as_string();

    use_dynamic_sub = !getPolygonFromString(poly_string, poly_);
  } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
    RCLCPP_INFO(
      logger_,
      "[%s]: Polygon points are not defined. Using dynamic subscription instead.",
      polygon_name_.c_str());
  }

  if (!getCommonParameters(
      polygon_sub_topic, polygon_pub_topic, footprint_topic, use_dynamic_sub))
  {
    if (use_dynamic_sub && polygon_sub_topic.empty() && footprint_topic.empty()) {
      RCLCPP_ERROR(
        logger_,
        "[%s]: Error while getting polygon parameters:"
        " static points and sub topic both not defined",
        polygon_name_.c_str());
    }
    return false;
  }

  return true;
}

void Polygon::createSubscription(std::string & polygon_sub_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!polygon_sub_topic.empty()) {
    RCLCPP_INFO(
      logger_,
      "[%s]: Subscribing on %s topic for polygon",
      polygon_name_.c_str(), polygon_sub_topic.c_str());
    rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();  // set to default
    if (polygon_subscribe_transient_local_) {
      polygon_qos.transient_local();
    }
    polygon_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      polygon_sub_topic, polygon_qos,
      std::bind(&Polygon::polygonCallback, this, std::placeholders::_1));
  }
}

void Polygon::updatePolygon(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
  std::size_t new_size = msg->polygon.points.size();

  if (new_size < 3) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Polygon should have at least 3 points",
      polygon_name_.c_str());
    return;
  }

  // Get the transform from PolygonStamped frame to base_frame_id_
  tf2::Stamped<tf2::Transform> tf_transform;
  if (
    !nav2_util::getTransform(
      msg->header.frame_id, base_frame_id_,
      transform_tolerance_, tf_buffer_, tf_transform))
  {
    return;
  }

  // Set main poly_ vertices first time
  poly_.resize(new_size);
  for (std::size_t i = 0; i < new_size; i++) {
    // Transform point coordinates from PolygonStamped frame -> to base frame
    tf2::Vector3 p_v3_s(msg->polygon.points[i].x, msg->polygon.points[i].y, 0.0);
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    // Fill poly_ array
    poly_[i] = {p_v3_b.x(), p_v3_b.y()};
  }

  // Store incoming polygon for further (possible) poly_ vertices corrections
  // from PolygonStamped frame -> to base frame
  polygon_ = *msg;
}

rcl_interfaces::msg::SetParametersResult
Polygon::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == polygon_name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

void Polygon::polygonCallback(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
  RCLCPP_INFO(
    logger_,
    "[%s]: Polygon shape update has been arrived",
    polygon_name_.c_str());
  updatePolygon(msg);
}

inline bool Polygon::isPointInside(const Point & point) const
{
  // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
  // Communications of the ACM 5.8 (1962): 434.
  // Implementation of ray crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  const int poly_size = poly_.size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result, initialized with already inverted value

  // Starting from the edge where the last point of polygon is connected to the first
  i = poly_size - 1;
  for (j = 0; j < poly_size; j++) {
    // Checking the edge only if given point is between edge boundaries by Y coordinates.
    // One of the condition should contain equality in order to exclude the edges
    // parallel to X+ ray.
    if ((point.y <= poly_[i].y) == (point.y > poly_[j].y)) {
      // Calculating the intersection coordinate of X+ ray
      const double x_inter = poly_[i].x +
        (point.y - poly_[i].y) * (poly_[j].x - poly_[i].x) /
        (poly_[j].y - poly_[i].y);
      // If intersection with checked edge is greater than point.x coordinate, inverting the result
      if (x_inter > point.x) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

bool Polygon::getPolygonFromString(
  std::string & poly_string,
  std::vector<Point> & polygon)
{
  std::string error;
  std::vector<std::vector<float>> vvf = nav2_util::parseVVF(poly_string, error);

  if (error != "") {
    RCLCPP_ERROR(
      logger_, "Error parsing polygon parameter %s: '%s'",
      poly_string.c_str(), error.c_str());
    return false;
  }

  // Check for minimum 4 points
  if (vvf.size() <= 3) {
    RCLCPP_ERROR(
      logger_,
      "Polygon must have at least three points.");
    return false;
  }
  for (unsigned int i = 0; i < vvf.size(); i++) {
    if (vvf[i].size() == 2) {
      Point point;
      point.x = vvf[i][0];
      point.y = vvf[i][1];
      polygon.push_back(point);
    } else {
      RCLCPP_ERROR(
        logger_,
        "Points in the polygon specification must be pairs of numbers"
        "Found a point with %d numbers.",
        static_cast<int>(vvf[i].size()));
      polygon.clear();
      return false;
    }
  }

  return true;
}

}  // namespace nav2_collision_monitor
