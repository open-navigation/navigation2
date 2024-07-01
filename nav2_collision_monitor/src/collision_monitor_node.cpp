// Copyright (c) 2022 Samsung R&D Institute Russia
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

#include "nav2_collision_monitor/collision_monitor_node.hpp"

#include <exception>
#include <utility>
#include <functional>

#include "tf2_ros/create_timer_ros.h"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

namespace nav2_collision_monitor
{

CollisionMonitor::CollisionMonitor(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("collision_monitor", "", options),
  process_active_(false), robot_action_prev_{DO_NOTHING, {-1.0, -1.0, -1.0}, ""},
  stop_stamp_{0, 0, get_clock()->get_clock_type()}, stop_pub_timeout_(1.0, 0.0)
{
}

CollisionMonitor::~CollisionMonitor()
{
  polygons_.clear();
  sources_.clear();
}

nav2_util::CallbackReturn
CollisionMonitor::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string cmd_vel_in_topic;
  std::string cmd_vel_out_topic;
  std::string state_topic;

  // Obtaining ROS parameters
  if (!getParameters(cmd_vel_in_topic, cmd_vel_out_topic, state_topic)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  cmd_vel_in_sub_ = std::make_unique<nav2_util::TwistSubscriber>(
    shared_from_this(),
    cmd_vel_in_topic,
    1,
    std::bind(&CollisionMonitor::cmdVelInCallbackUnstamped, this, std::placeholders::_1),
    std::bind(&CollisionMonitor::cmdVelInCallbackStamped, this, std::placeholders::_1));

  auto node = shared_from_this();
  cmd_vel_out_pub_ = std::make_unique<nav2_util::TwistPublisher>(node, cmd_vel_out_topic, 1);

  if (!state_topic.empty()) {
    state_pub_ = this->create_publisher<nav2_msgs::msg::CollisionMonitorState>(
      state_topic, 1);
  }

  collision_points_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/collision_points_marker", 1);

  nav2_util::declare_parameter_if_not_declared(
    node, "use_realtime_priority", rclcpp::ParameterValue(false));
  bool use_realtime_priority = false;
  node->get_parameter("use_realtime_priority", use_realtime_priority);
  if (use_realtime_priority) {
    try {
      nav2_util::setSoftRealTimePriority();
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publisher
  cmd_vel_out_pub_->on_activate();
  if (state_pub_) {
    state_pub_->on_activate();
  }
  collision_points_marker_pub_->on_activate();

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->activate();
  }

  // Since polygons are being published when cmd_vel_in appears,
  // we need to publish polygons first time to display them at startup
  publishPolygons();

  // Activating main worker
  process_active_ = true;

  // Creating bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Deactivating main worker
  process_active_ = false;

  // Reset action type to default after worker deactivating
  robot_action_prev_ = {DO_NOTHING, {-1.0, -1.0, -1.0}, ""};

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->deactivate();
  }

  // Deactivating lifecycle publishers
  cmd_vel_out_pub_->on_deactivate();
  if (state_pub_) {
    state_pub_->on_deactivate();
  }
  collision_points_marker_pub_->on_deactivate();

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  cmd_vel_in_sub_.reset();
  cmd_vel_out_pub_.reset();
  state_pub_.reset();
  collision_points_marker_pub_.reset();

  polygons_.clear();
  sources_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

void CollisionMonitor::cmdVelInCallbackStamped(geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(*msg)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }

  process({msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z}, msg->header);
}

void CollisionMonitor::cmdVelInCallbackUnstamped(geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto twist_stamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_stamped->twist = *msg;
  cmdVelInCallbackStamped(twist_stamped);
}

void CollisionMonitor::publishVelocity(
  const Action & robot_action, const std_msgs::msg::Header & header)
{
  if (robot_action.req_vel.isZero()) {
    if (!robot_action_prev_.req_vel.isZero()) {
      // Robot just stopped: saving stop timestamp and continue
      stop_stamp_ = this->now();
    } else if (this->now() - stop_stamp_ > stop_pub_timeout_) {
      // More than stop_pub_timeout_ passed after robot has been stopped.
      // Cease publishing output cmd_vel.
      return;
    }
  }

  auto cmd_vel_out_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel_out_msg->header = header;
  cmd_vel_out_msg->twist.linear.x = robot_action.req_vel.x;
  cmd_vel_out_msg->twist.linear.y = robot_action.req_vel.y;
  cmd_vel_out_msg->twist.angular.z = robot_action.req_vel.tw;
  // linear.z, angular.x and angular.y will remain 0.0

  cmd_vel_out_pub_->publish(std::move(cmd_vel_out_msg));
}

bool CollisionMonitor::getParameters(
  std::string & cmd_vel_in_topic,
  std::string & cmd_vel_out_topic,
  std::string & state_topic)
{
  std::string base_frame_id, odom_frame_id;
  tf2::Duration transform_tolerance;
  rclcpp::Duration source_timeout(2.0, 0.0);

  auto node = shared_from_this();

  nav2_util::declare_parameter_if_not_declared(
    node, "cmd_vel_in_topic", rclcpp::ParameterValue("cmd_vel_smoothed"));
  cmd_vel_in_topic = get_parameter("cmd_vel_in_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "cmd_vel_out_topic", rclcpp::ParameterValue("cmd_vel"));
  cmd_vel_out_topic = get_parameter("cmd_vel_out_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "state_topic", rclcpp::ParameterValue(""));
  state_topic = get_parameter("state_topic").as_string();

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

  nav2_util::declare_parameter_if_not_declared(
    node, "stop_pub_timeout", rclcpp::ParameterValue(1.0));
  stop_pub_timeout_ =
    rclcpp::Duration::from_seconds(get_parameter("stop_pub_timeout").as_double());

  if (!configurePolygons(base_frame_id, transform_tolerance)) {
    return false;
  }

  if (
    !configureSources(
      base_frame_id, odom_frame_id, transform_tolerance, source_timeout, base_shift_correction))
  {
    return false;
  }

  return true;
}

bool CollisionMonitor::configurePolygons(
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
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionMonitor::configureSources(
  const std::string & base_frame_id,
  const std::string & odom_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
{
  try {
    auto node = shared_from_this();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
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

void CollisionMonitor::process(const Velocity & cmd_vel_in, const std_msgs::msg::Header & header)
{
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->now();

  // Do nothing if main worker in non-active state
  if (!process_active_) {
    return;
  }

  // Points array collected from different data sources in a robot base frame
  std::vector<Point> collision_points;

  // By default - there is no action
  Action robot_action{DO_NOTHING, cmd_vel_in, ""};
  // Polygon causing robot action (if any)
  std::shared_ptr<Polygon> action_polygon;

  // Fill collision_points array from different data sources
  for (std::shared_ptr<Source> source : sources_) {
    if (source->getEnabled()) {
      if (!source->getData(curr_time, collision_points) &&
        source->getSourceTimeout().seconds() != 0.0)
      {
        action_polygon = nullptr;
        robot_action.polygon_name = "invalid source";
        robot_action.action_type = STOP;
        robot_action.req_vel.x = 0.0;
        robot_action.req_vel.y = 0.0;
        robot_action.req_vel.tw = 0.0;
        break;
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
    if (robot_action.action_type == STOP) {
      // If robot already should stop, do nothing
      break;
    }

    // Update polygon coordinates
    polygon->updatePolygon(cmd_vel_in);

    const ActionType at = polygon->getActionType();
    if (at == STOP || at == SLOWDOWN || at == LIMIT) {
      // Process STOP/SLOWDOWN for the selected polygon
      if (processStopSlowdownLimit(polygon, collision_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    } else if (at == APPROACH) {
      // Process APPROACH for the selected polygon
      if (processApproach(polygon, collision_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    }
  }

  if (robot_action.polygon_name != robot_action_prev_.polygon_name) {
    // Report changed robot behavior
    notifyActionState(robot_action, action_polygon);
  }

  // Publish required robot velocity
  publishVelocity(robot_action, header);

  // Publish polygons for better visualization
  publishPolygons();

  robot_action_prev_ = robot_action;
}

bool CollisionMonitor::processStopSlowdownLimit(
  const std::shared_ptr<Polygon> polygon,
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  if (!polygon->isShapeSet()) {
    return false;
  }
  polygon->newCollisionPoints(collision_points, get_clock()->now());
  if (polygon->shouldApplyAction()) {
    if (polygon->getActionType() == STOP) {
      // Setting up zero velocity for STOP model
      robot_action.polygon_name = polygon->getName();
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      return true;
    } else if (polygon->getActionType() == SLOWDOWN) {
      const Velocity safe_vel = velocity * polygon->getSlowdownRatio();
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.polygon_name = polygon->getName();
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
        return true;
      }
    } else {  // Limit
      // Compute linear velocity
      const double linear_vel = std::hypot(velocity.x, velocity.y);  // absolute
      Velocity safe_vel;
      double ratio = 1.0;
      if (linear_vel != 0.0) {
        ratio = std::clamp(polygon->getLinearLimit() / linear_vel, 0.0, 1.0);
      }
      safe_vel.x = velocity.x * ratio;
      safe_vel.y = velocity.y * ratio;
      safe_vel.tw = std::clamp(
        velocity.tw, -polygon->getAngularLimit(), polygon->getAngularLimit());
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.polygon_name = polygon->getName();
        robot_action.action_type = LIMIT;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
  }

  return false;
}

bool CollisionMonitor::processApproach(
  const std::shared_ptr<Polygon> polygon,
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  if (!polygon->isShapeSet()) {
    return false;
  }

  // Obtain time before a collision
  const double collision_time = polygon->getCollisionTime(collision_points, velocity);
  if (collision_time >= 0.0) {
    // If collision will occurr, reduce robot speed
    const double change_ratio = collision_time / polygon->getTimeBeforeCollision();
    const Velocity safe_vel = velocity * change_ratio;
    // Check that currently calculated velocity is safer than
    // chosen for previous shapes one
    if (safe_vel < robot_action.req_vel) {
      robot_action.polygon_name = polygon->getName();
      robot_action.action_type = APPROACH;
      robot_action.req_vel = safe_vel;
      return true;
    }
  }

  return false;
}

void CollisionMonitor::notifyActionState(
  const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const
{
  if (robot_action.action_type == STOP) {
    if (robot_action.polygon_name == "invalid source") {
      RCLCPP_WARN(
        get_logger(),
        "Robot to stop due to invalid source."
        " Either due to data not published yet, or to lack of new data received within the"
        " sensor timeout, or if impossible to transform data to base frame");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Robot to stop due to %s polygon",
        action_polygon->getName().c_str());
    }
  } else if (robot_action.action_type == SLOWDOWN) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to slowdown for %f percents due to %s polygon",
      action_polygon->getSlowdownRatio() * 100,
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == LIMIT) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to limit speed due to %s polygon",
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == APPROACH) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to approach for %f seconds away from collision",
      action_polygon->getTimeBeforeCollision());
  } else {  // robot_action.action_type == DO_NOTHING
    RCLCPP_INFO(
      get_logger(),
      "Robot to continue normal operation");
  }

  if (state_pub_) {
    std::unique_ptr<nav2_msgs::msg::CollisionMonitorState> state_msg =
      std::make_unique<nav2_msgs::msg::CollisionMonitorState>();
    state_msg->polygon_name = robot_action.polygon_name;
    state_msg->action_type = robot_action.action_type;

    state_pub_->publish(std::move(state_msg));
  }
}

void CollisionMonitor::publishPolygons() const
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
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionMonitor)
