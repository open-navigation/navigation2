// Copyright (c) 2019 Intel Corporation
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

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_controller/controller_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("controller_server", "", options),
  progress_checker_loader_("nav2_core", "nav2_core::ProgressChecker"),
  default_progress_checker_ids_{"progress_checker"},
  default_progress_checker_types_{"nav2_controller::SimpleProgressChecker"},
  goal_checker_loader_("nav2_core", "nav2_core::GoalChecker"),
  default_goal_checker_ids_{"goal_checker"},
  default_goal_checker_types_{"nav2_controller::SimpleGoalChecker"},
  lp_loader_("nav2_core", "nav2_core::Controller"),
  default_ids_{"FollowPath"},
  default_types_{"dwb_core::DWBLocalPlanner"}
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("action_server_result_timeout", 10.0);

  declare_parameter("progress_checker_plugins", default_progress_checker_ids_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("use_realtime_priority", rclcpp::ParameterValue(false));

  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap",
    get_parameter("use_sim_time").as_bool());
}

ControllerServer::~ControllerServer()
{
  progress_checkers_.clear();
  goal_checkers_.clear();
  controllers_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
ControllerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  RCLCPP_INFO(get_logger(), "getting progress checker plugins..");
  get_parameter("progress_checker_plugins", progress_checker_ids_);
  if (progress_checker_ids_ == default_progress_checker_ids_) {
    for (size_t i = 0; i < default_progress_checker_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_progress_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_progress_checker_types_[i]));
    }
  }

  RCLCPP_INFO(get_logger(), "getting goal checker plugins..");
  get_parameter("goal_checker_plugins", goal_checker_ids_);
  if (goal_checker_ids_ == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_goal_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }

  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  controller_types_.resize(controller_ids_.size());
  goal_checker_types_.resize(goal_checker_ids_.size());
  progress_checker_types_.resize(progress_checker_ids_.size());

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  get_parameter("failure_tolerance", failure_tolerance_);
  get_parameter("use_realtime_priority", use_realtime_priority_);

  costmap_ros_->configure();
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  for (size_t i = 0; i != progress_checker_ids_.size(); i++) {
    try {
      progress_checker_types_[i] = nav2_util::get_plugin_type_param(
        node, progress_checker_ids_[i]);
      nav2_core::ProgressChecker::Ptr progress_checker =
        progress_checker_loader_.createUniqueInstance(progress_checker_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created progress_checker : %s of type %s",
        progress_checker_ids_[i].c_str(), progress_checker_types_[i].c_str());
      progress_checker->initialize(node, progress_checker_ids_[i]);
      progress_checkers_.insert({progress_checker_ids_[i], progress_checker});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create progress_checker. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != progress_checker_ids_.size(); i++) {
    progress_checker_ids_concat_ += progress_checker_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s progress checkers available.", progress_checker_ids_concat_.c_str());

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] = nav2_util::get_plugin_type_param(node, goal_checker_ids_[i]);
      nav2_core::GoalChecker::Ptr goal_checker =
        goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created goal checker : %s of type %s",
        goal_checker_ids_[i].c_str(), goal_checker_types_[i].c_str());
      goal_checker->initialize(node, goal_checker_ids_[i], costmap_ros_);
      goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create goal checker. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s goal checkers available.", goal_checker_ids_concat_.c_str());

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] = nav2_util::get_plugin_type_param(node, controller_ids_[i]);
      nav2_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created controller : %s of type %s",
        controller_ids_[i].c_str(), controller_types_[i].c_str());
      controller->configure(
        node, controller_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create controller. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  vel_publisher_ = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel", 1);

  double action_server_result_timeout;
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  // Create the action server that we implement with our followPath method
  // This may throw due to real-time prioritzation if user doesn't have real-time permissions
  try {
    action_server_ = std::make_unique<ActionServer>(
      shared_from_this(),
      "follow_path",
      std::bind(&ControllerServer::computeControl, this),
      nullptr,
      std::chrono::milliseconds(500),
      true /*spin thread*/, server_options, use_realtime_priority_ /*soft realtime*/);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(get_logger(), "Error creating action server! %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Set subscribtion to the speed limiting topic
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10),
    std::bind(&ControllerServer::speedLimitCallback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  const auto costmap_ros_state = costmap_ros_->activate();
  if (costmap_ros_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return nav2_util::CallbackReturn::FAILURE;
  }
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->activate();
  }
  vel_publisher_->on_activate();
  action_server_->activate();

  auto node = shared_from_this();
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ControllerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->deactivate();
  }

  /*
   * The costmap is also a lifecycle node, so it may have already fired on_deactivate
   * via rcl preshutdown cb. Despite the rclcpp docs saying on_shutdown callbacks fire
   * in the order added, the preshutdown callbacks clearly don't per se, due to using an
   * unordered_set iteration. Once this issue is resolved, we can maybe make a stronger
   * ordering assumption: https://github.com/ros2/rclcpp/issues/2096
   */
  costmap_ros_->deactivate();

  publishZeroVelocity();
  vel_publisher_->on_deactivate();
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->cleanup();
  }
  controllers_.clear();

  goal_checkers_.clear();
  progress_checkers_.clear();

  costmap_ros_->cleanup();


  // Release any allocated resources
  action_server_.reset();
  odom_sub_.reset();
  costmap_thread_.reset();
  vel_publisher_.reset();
  speed_limit_sub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool ControllerServer::findControllerId(
  const std::string & c_name,
  std::string & current_controller)
{
  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No controller was specified in action call."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with controller name %s, "
        "which does not exist. Available controllers are: %s.",
        c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

bool ControllerServer::findGoalCheckerId(
  const std::string & c_name,
  std::string & current_goal_checker)
{
  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No goal checker was specified in parameter 'current_goal_checker'."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", goal_checker_ids_concat_.c_str());
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with goal_checker name %s in parameter"
        " 'current_goal_checker', which does not exist. Available goal checkers are: %s.",
        c_name.c_str(), goal_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected goal checker: %s.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

bool ControllerServer::findProgressCheckerId(
  const std::string & c_name,
  std::string & current_progress_checker)
{
  if (progress_checkers_.find(c_name) == progress_checkers_.end()) {
    if (progress_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No progress checker was specified in parameter 'current_progress_checker'."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", progress_checker_ids_concat_.c_str());
      current_progress_checker = progress_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with progress_checker name %s in parameter"
        " 'current_progress_checker', which does not exist. Available progress checkers are: %s.",
        c_name.c_str(), progress_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected progress checker: %s.", c_name.c_str());
    current_progress_checker = c_name;
  }

  return true;
}

void ControllerServer::computeControl()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try {
    std::string c_name = action_server_->get_current_goal()->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      throw nav2_core::InvalidController("Failed to find controller name: " + c_name);
    }

    std::string gc_name = action_server_->get_current_goal()->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      throw nav2_core::ControllerException("Failed to find goal checker name: " + gc_name);
    }

    std::string pc_name = action_server_->get_current_goal()->progress_checker_id;
    std::string current_progress_checker;
    if (findProgressCheckerId(pc_name, current_progress_checker)) {
      current_progress_checker_ = current_progress_checker;
    } else {
      throw nav2_core::ControllerException("Failed to find progress checker name: " + pc_name);
    }

    setPlannerPath(action_server_->get_current_goal()->path);
    progress_checkers_[current_progress_checker_]->reset();

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok()) {
      auto start_time = this->now();

      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        if (controllers_[current_controller_]->cancel()) {
          RCLCPP_INFO(get_logger(), "Cancellation was successful. Stopping the robot.");
          action_server_->terminate_all();
          publishZeroVelocity();
          return;
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000, "Waiting for the controller to finish cancellation");
        }
      }

      // Don't compute a trajectory until costmap is valid (after clear costmap)
      rclcpp::Rate r(100);
      while (!costmap_ros_->isCurrent()) {
        r.sleep();
      }

      updateGlobalPath();

      computeAndPublishVelocity();

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      auto cycle_duration = this->now() - start_time;
      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
          get_logger(),
          "Control loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz.",
          controller_frequency_, 1 / cycle_duration.seconds());
      }
    }
  } catch (nav2_core::InvalidController & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::INVALID_CONTROLLER;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::ControllerTFError & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::TF_ERROR;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::NoValidControl & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::NO_VALID_CONTROL;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::FailedToMakeProgress & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::FAILED_TO_MAKE_PROGRESS;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::PatienceExceeded & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::PATIENCE_EXCEEDED;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::InvalidPath & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::INVALID_PATH;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::ControllerException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::UNKNOWN;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::UNKNOWN;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_->succeeded_current();
}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  RCLCPP_DEBUG(
    get_logger(),
    "Providing path to the controller %s", current_controller_.c_str());
  if (path.poses.empty()) {
    throw nav2_core::InvalidPath("Path is empty.");
  }
  controllers_[current_controller_]->setPlan(path);

  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  goal_checkers_[current_goal_checker_]->reset();

  RCLCPP_DEBUG(
    get_logger(), "Path end point is (%.2f, %.2f)",
    end_pose_.pose.position.x, end_pose_.pose.position.y);

  current_path_ = path;
}

void ControllerServer::computeAndPublishVelocity()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    throw nav2_core::ControllerTFError("Failed to obtain robot pose");
  }

  if (!progress_checkers_[current_progress_checker_]->check(pose)) {
    throw nav2_core::FailedToMakeProgress("Failed to make progress");
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());

  geometry_msgs::msg::TwistStamped cmd_vel_2d;

  try {
    cmd_vel_2d =
      controllers_[current_controller_]->computeVelocityCommands(
      pose,
      nav_2d_utils::twist2Dto3D(twist),
      goal_checkers_[current_goal_checker_].get());
    last_valid_cmd_time_ = now();
    cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
    cmd_vel_2d.header.stamp = last_valid_cmd_time_;
    // Only no valid control exception types are valid to attempt to have control patience, as
    // other types will not be resolved with more attempts
  } catch (nav2_core::NoValidControl & e) {
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      cmd_vel_2d.twist.angular.x = 0;
      cmd_vel_2d.twist.angular.y = 0;
      cmd_vel_2d.twist.angular.z = 0;
      cmd_vel_2d.twist.linear.x = 0;
      cmd_vel_2d.twist.linear.y = 0;
      cmd_vel_2d.twist.linear.z = 0;
      cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel_2d.header.stamp = now();
      if ((now() - last_valid_cmd_time_).seconds() > failure_tolerance_ &&
        failure_tolerance_ != -1.0)
      {
        throw nav2_core::PatienceExceeded("Controller patience exceeded");
      }
    } else {
      throw nav2_core::NoValidControl(e.what());
    }
  }

  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

  // Find the closest pose to current pose on global path
  nav_msgs::msg::Path & current_path = current_path_;
  auto find_closest_pose_idx =
    [&pose, &current_path]() {
      size_t closest_pose_idx = 0;
      double curr_min_dist = std::numeric_limits<double>::max();
      for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
        double curr_dist = nav2_util::geometry_utils::euclidean_distance(
          pose, current_path.poses[curr_idx]);
        if (curr_dist < curr_min_dist) {
          curr_min_dist = curr_dist;
          closest_pose_idx = curr_idx;
        }
      }
      return closest_pose_idx;
    };

  feedback->distance_to_goal =
    nav2_util::geometry_utils::calculate_path_length(current_path_, find_closest_pose_idx());
  action_server_->publish_feedback(feedback);

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);
}

void ControllerServer::updateGlobalPath()
{
  if (action_server_->is_preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Passing new path to controller.");
    auto goal = action_server_->accept_pending_goal();
    std::string current_controller;
    if (findControllerId(goal->controller_id, current_controller)) {
      current_controller_ = current_controller;
    } else {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid controller %s requested.",
        goal->controller_id.c_str());
      action_server_->terminate_current();
      return;
    }
    std::string current_goal_checker;
    if (findGoalCheckerId(goal->goal_checker_id, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid goal checker %s requested.",
        goal->goal_checker_id.c_str());
      action_server_->terminate_current();
      return;
    }
    std::string current_progress_checker;
    if (findProgressCheckerId(goal->progress_checker_id, current_progress_checker)) {
      if (current_progress_checker_ != current_progress_checker) {
        RCLCPP_INFO(
          get_logger(), "Change of progress checker %s requested, resetting it",
          goal->progress_checker_id.c_str());
        current_progress_checker_ = current_progress_checker;
        progress_checkers_[current_progress_checker_]->reset();
      }
    } else {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid progress checker %s requested.",
        goal->progress_checker_id.c_str());
      action_server_->terminate_current();
      return;
    }
    setPlannerPath(goal->path);
  }
}

void ControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>(velocity);
  if (vel_publisher_->is_activated() && vel_publisher_->get_subscription_count() > 0) {
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

void ControllerServer::publishZeroVelocity()
{
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = costmap_ros_->getBaseFrameID();
  velocity.header.stamp = now();
  publishVelocity(velocity);

  // Reset the state of the controllers after the task has ended
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->reset();
  }
}

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    return false;
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());
  geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(twist);

  geometry_msgs::msg::PoseStamped transformed_end_pose;
  rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
  nav_2d_utils::transformPose(
    costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
    end_pose_, transformed_end_pose, tolerance);

  return goal_checkers_[current_goal_checker_]->isGoalReached(
    pose.pose, transformed_end_pose.pose,
    velocity);
}

bool ControllerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

void ControllerServer::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
{
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
}

rcl_interfaces::msg::SetParametersResult
ControllerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // as they handle parameter changes themselves and don't need to lock the mutex
    if (name.find('.') != std::string::npos) {
      continue;
    }

    if (!dynamic_params_lock_.try_lock()) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to dynamically change Parameters while the controller is currently running");
      result.successful = false;
      result.reason =
        "Unable to dynamically change Parameters while the controller is currently running";
      return result;
    }

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "min_x_velocity_threshold") {
        min_x_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_y_velocity_threshold") {
        min_y_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_theta_velocity_threshold") {
        min_theta_velocity_threshold_ = parameter.as_double();
      } else if (name == "failure_tolerance") {
        failure_tolerance_ = parameter.as_double();
      }
    }

    dynamic_params_lock_.unlock();
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_controller::ControllerServer)
