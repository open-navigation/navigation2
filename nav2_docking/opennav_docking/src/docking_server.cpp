// Copyright (c) 2024 Open Navigation LLC
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

#include "angles/angles.h"
#include "opennav_docking/docking_server.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace opennav_docking
{

DockingServer::DockingServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("docking_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());

  declare_parameter("controller_frequency", 50.0);
  declare_parameter("initial_perception_timeout", 5.0);
  declare_parameter("wait_charge_timeout", 5.0);
  declare_parameter("dock_approach_timeout", 30.0);
  declare_parameter("undock_linear_tolerance", 0.05);
  declare_parameter("undock_angular_tolerance", 0.05);
  declare_parameter("max_retries", 3);
  declare_parameter("base_frame", "base_link");
  declare_parameter("fixed_frame", "odom");
  declare_parameter("dock_backwards", false);
  declare_parameter("dock_prestaging_tolerance", 0.5);
}

nav2_util::CallbackReturn
DockingServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("initial_perception_timeout", initial_perception_timeout_);
  get_parameter("wait_charge_timeout", wait_charge_timeout_);
  get_parameter("dock_approach_timeout", dock_approach_timeout_);
  get_parameter("undock_linear_tolerance", undock_linear_tolerance_);
  get_parameter("undock_angular_tolerance", undock_angular_tolerance_);
  get_parameter("max_retries", max_retries_);
  get_parameter("base_frame", base_frame_);
  get_parameter("fixed_frame", fixed_frame_);
  get_parameter("dock_backwards", dock_backwards_);
  get_parameter("dock_prestaging_tolerance", dock_prestaging_tolerance_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  vel_publisher_ = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel", 1);
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  double action_server_result_timeout;
  nav2_util::declare_parameter_if_not_declared(
    node, "action_server_result_timeout", rclcpp::ParameterValue(10.0));
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  // Create the action servers for dock / undock
  docking_action_server_ = std::make_unique<DockingActionServer>(
    node, "dock_robot",
    std::bind(&DockingServer::dockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  undocking_action_server_ = std::make_unique<UndockingActionServer>(
    node, "undock_robot",
    std::bind(&DockingServer::undockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  // Create composed utilities
  controller_ = std::make_unique<Controller>(node);
  navigator_ = std::make_unique<Navigator>(node);
  dock_db_ = std::make_unique<DockDatabase>();
  if (!dock_db_->initialize(node, tf2_buffer_)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());

  auto node = shared_from_this();

  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
  dock_db_->activate();
  navigator_->activate();
  vel_publisher_->on_activate();
  docking_action_server_->activate();
  undocking_action_server_->activate();
  curr_dock_type_.clear();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&DockingServer::dynamicParametersCallback, this, _1));

  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  docking_action_server_->deactivate();
  undocking_action_server_->deactivate();
  dock_db_->deactivate();
  navigator_->deactivate();
  vel_publisher_->on_deactivate();

  dyn_params_handler_.reset();
  tf2_listener_.reset();

  // Destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  tf2_buffer_.reset();
  docking_action_server_.reset();
  undocking_action_server_.reset();
  dock_db_.reset();
  navigator_.reset();
  curr_dock_type_.clear();
  controller_.reset();
  vel_publisher_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename ActionT>
void DockingServer::getPreemptedGoalIfRequested(
  typename std::shared_ptr<const typename ActionT::Goal> goal,
  const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfCancelled(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was cancelled. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfPreempted(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_preempt_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was preempted. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

void DockingServer::dockRobot()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = docking_action_server_->get_current_goal();
  auto result = std::make_shared<DockRobot::Result>();
  result->success = false;

  if (!docking_action_server_ || !docking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot")) {
    docking_action_server_->terminate_all();
    return;
  }

  getPreemptedGoalIfRequested(goal, docking_action_server_);
  Dock * dock{nullptr};
  num_retries_ = 0;

  try {
    // Get dock (instance and plugin information) from request
    if (goal->use_dock_id) {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at charger %s.", goal->dock_id.c_str());
      dock = dock_db_->findDock(goal->dock_id);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at charger at position (%0.2f, %0.2f).",
        goal->dock_pose.pose.position.x, goal->dock_pose.pose.position.y);
      dock = generateGoalDock(goal);
    }

    // Check if the robot is docked or charging before proceeding
    if (dock->plugin->isDocked() || dock->plugin->isCharging()) {
      RCLCPP_INFO(get_logger(), "Robot is already charging, no need to dock");
      return;
    }

    // Send robot to its staging pose
    publishDockingFeedback(DockRobot::Feedback::NAV_TO_STAGING_POSE);
    const auto initial_staging_pose = dock->getStagingPose();
    const auto robot_pose = getRobotPoseInFrame(
      initial_staging_pose.header.frame_id);
    if (!goal->navigate_to_staging_pose ||
      utils::l2Norm(robot_pose.pose, initial_staging_pose.pose) < dock_prestaging_tolerance_)
    {
      RCLCPP_INFO(get_logger(), "Robot already within pre-staging pose tolerance for dock");
    } else {
      navigator_->goToPose(
        initial_staging_pose, rclcpp::Duration::from_seconds(goal->max_staging_time));
      RCLCPP_INFO(get_logger(), "Successful navigation to staging pose");
    }

    // Construct initial estimate of where the dock is located in fixed_frame
    auto dock_pose = utils::getDockPoseStamped(dock, rclcpp::Time(0));
    tf2_buffer_->transform(dock_pose, dock_pose, fixed_frame_);

    // Get initial detection of dock before proceeding to move
    doInitialPerception(dock, dock_pose);
    RCLCPP_INFO(get_logger(), "Successful initial dock detection");

    // Docking control loop: while not docked, run controller
    rclcpp::Time dock_contact_time;
    while (rclcpp::ok()) {
      try {
        // Approach the dock using control law
        if (approachDock(dock, dock_pose)) {
          // We are docked, wait for charging to begin
          RCLCPP_INFO(get_logger(), "Made contact with dock, waiting for charge to start");
          if (waitForCharge(dock)) {
            RCLCPP_INFO(get_logger(), "Robot is charging!");
            result->success = true;
            result->num_retries = num_retries_;
            stashDockData(goal->use_dock_id, dock, true);
            publishZeroVelocity();
            docking_action_server_->succeeded_current(result);
            return;
          }
        }

        // Cancelled, preempted, or shutting down (recoverable errors throw DockingException)
        stashDockData(goal->use_dock_id, dock, false);
        publishZeroVelocity();
        docking_action_server_->terminate_all(result);
        return;
      } catch (opennav_docking_core::DockingException & e) {
        if (++num_retries_ > max_retries_) {
          RCLCPP_ERROR(get_logger(), "Failed to dock, all retries have been used");
          throw;
        }
        RCLCPP_WARN(get_logger(), "Docking failed, will retry: %s", e.what());
      }

      // Reset to staging pose to try again
      if (!resetApproach(dock->getStagingPose())) {
        // Cancelled, preempted, or shutting down
        stashDockData(goal->use_dock_id, dock, false);
        publishZeroVelocity();
        docking_action_server_->terminate_all(result);
        return;
      }
      RCLCPP_INFO(get_logger(), "Returned to staging pose, attempting docking again");
    }
  } catch (const tf2::TransformException & e) {
    result->error_msg = std::string("Transform error: ") + e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (opennav_docking_core::DockNotInDB & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::DOCK_NOT_IN_DB;
  } catch (opennav_docking_core::DockNotValid & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToStage & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::FAILED_TO_STAGE;
  } catch (opennav_docking_core::FailedToDetectDock & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::FAILED_TO_DETECT_DOCK;
  } catch (opennav_docking_core::FailedToControl & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::FailedToCharge & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::FAILED_TO_CHARGE;
  } catch (opennav_docking_core::DockingException & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    result->error_code = DockRobot::Result::UNKNOWN;
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
  }

  // Store dock state for later undocking and delete temp dock, if applicable
  stashDockData(goal->use_dock_id, dock, false);
  result->num_retries = num_retries_;
  publishZeroVelocity();
  docking_action_server_->terminate_current(result);
}

void DockingServer::stashDockData(bool use_dock_id, Dock * dock, bool successful)
{
  if (dock && successful) {
    curr_dock_type_ = dock->type;
  }

  if (!use_dock_id && dock) {
    delete dock;
    dock = nullptr;
  }
}

Dock * DockingServer::generateGoalDock(std::shared_ptr<const DockRobot::Goal> goal)
{
  auto dock = new Dock();
  dock->frame = goal->dock_pose.header.frame_id;
  dock->pose = goal->dock_pose.pose;
  dock->type = goal->dock_type;
  dock->plugin = dock_db_->findDockPlugin(dock->type);
  return dock;
}

void DockingServer::doInitialPerception(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  publishDockingFeedback(DockRobot::Feedback::INITIAL_PERCEPTION);
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(initial_perception_timeout_);
  while (!dock->plugin->getRefinedPose(dock_pose)) {
    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToDetectDock("Failed initial dock detection");
    }

    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return;
    }

    loop_rate.sleep();
  }
}

bool DockingServer::approachDock(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(dock_approach_timeout_);
  while (rclcpp::ok()) {
    publishDockingFeedback(DockRobot::Feedback::CONTROLLING);

    // Stop and report success if connected to dock
    if (dock->plugin->isDocked() || dock->plugin->isCharging()) {
      return true;
    }

    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    // Update perception
    if (!dock->plugin->getRefinedPose(dock_pose)) {
      throw opennav_docking_core::FailedToDetectDock("Failed dock detection");
    }

    // Transform target_pose into base_link frame
    geometry_msgs::msg::PoseStamped target_pose = dock_pose;
    target_pose.header.stamp = rclcpp::Time(0);

    // The control law can get jittery when close to the end when atan2's can explode.
    // Thus, we backward project the controller's target pose a little bit after the
    // dock so that the robot never gets to the end of the spiral before its in contact
    // with the dock to stop the docking procedure.
    const double backward_projection = 0.25;
    const double yaw = tf2::getYaw(target_pose.pose.orientation);
    target_pose.pose.position.x += cos(yaw) * backward_projection;
    target_pose.pose.position.y += sin(yaw) * backward_projection;
    tf2_buffer_->transform(target_pose, target_pose, base_frame_);

    // Compute and publish controls
    auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
    command->header.stamp = now();
    if (!controller_->computeVelocityCommand(target_pose.pose, command->twist, dock_backwards_)) {
      throw opennav_docking_core::FailedToControl("Failed to get control");
    }
    vel_publisher_->publish(std::move(command));

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToControl(
              "Timed out approaching dock; dock nor charging detected");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::waitForCharge(Dock * dock)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(wait_charge_timeout_);
  while (rclcpp::ok()) {
    publishDockingFeedback(DockRobot::Feedback::WAIT_FOR_CHARGE);

    if (dock->plugin->isCharging()) {
      return true;
    }

    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToCharge("Timed out waiting for charge to start");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::resetApproach(const geometry_msgs::msg::PoseStamped & staging_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(dock_approach_timeout_);
  while (rclcpp::ok()) {
    publishDockingFeedback(DockRobot::Feedback::INITIAL_PERCEPTION);

    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    // Compute and publish command
    auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
    command->header.stamp = now();
    if (getCommandToPose(
        command->twist, staging_pose, undock_linear_tolerance_, undock_angular_tolerance_,
        !dock_backwards_))
    {
      return true;
    }
    vel_publisher_->publish(std::move(command));

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToControl("Timed out resetting dock approach");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::getCommandToPose(
  geometry_msgs::msg::Twist & cmd, const geometry_msgs::msg::PoseStamped & pose,
  double linear_tolerance, double angular_tolerance, bool backward)
{
  // Reset command to zero velocity
  cmd.linear.x = 0;
  cmd.angular.z = 0;

  // Determine if we have reached pose yet & stop
  geometry_msgs::msg::PoseStamped robot_pose = getRobotPoseInFrame(pose.header.frame_id);
  const double dist = std::hypot(
    robot_pose.pose.position.x - pose.pose.position.x,
    robot_pose.pose.position.y - pose.pose.position.y);
  const double yaw = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(pose.pose.orientation));
  if (dist < linear_tolerance && abs(yaw) < angular_tolerance) {
    return true;
  }

  // Transform target_pose into base_link frame
  geometry_msgs::msg::PoseStamped target_pose = pose;
  target_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(target_pose, target_pose, base_frame_);

  // Compute velocity command
  if (!controller_->computeVelocityCommand(target_pose.pose, cmd, backward)) {
    throw opennav_docking_core::FailedToControl("Failed to get control");
  }

  // Command is valid, but target is not reached
  return false;
}

void DockingServer::undockRobot()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = undocking_action_server_->get_current_goal();
  auto result = std::make_shared<UndockRobot::Result>();
  result->success = false;

  if (!undocking_action_server_ || !undocking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled(undocking_action_server_, "undock_robot")) {
    undocking_action_server_->terminate_all(result);
    return;
  }

  getPreemptedGoalIfRequested(goal, undocking_action_server_);
  auto max_duration = rclcpp::Duration::from_seconds(goal->max_undocking_time);

  try {
    // Get dock plugin information from request or docked state, reset state.
    std::string dock_type = curr_dock_type_;
    if (!goal->dock_type.empty()) {
      dock_type = goal->dock_type;
    }

    ChargingDock::Ptr dock = dock_db_->findDockPlugin(dock_type);
    if (!dock) {
      throw opennav_docking_core::DockNotValid("No dock information to undock from!");
    }
    RCLCPP_INFO(
      get_logger(),
      "Attempting to undock robot from charger of type %s.", dock->getName().c_str());

    // Check if the robot is docked before proceeding
    if (!dock->isDocked()) {
      RCLCPP_INFO(get_logger(), "Robot is not in the charger, no need to undock");
      return;
    }

    // Get "dock pose" by finding the robot pose
    geometry_msgs::msg::PoseStamped dock_pose = getRobotPoseInFrame(fixed_frame_);

    // Get staging pose (in fixed frame)
    geometry_msgs::msg::PoseStamped staging_pose =
      dock->getStagingPose(dock_pose.pose, dock_pose.header.frame_id);

    // Control robot to staging pose
    rclcpp::Time loop_start = this->now();
    while (rclcpp::ok()) {
      // Stop if we exceed max duration
      auto timeout = rclcpp::Duration::from_seconds(goal->max_undocking_time);
      if (this->now() - loop_start > timeout) {
        throw opennav_docking_core::FailedToControl("Undocking timed out");
      }

      // Stop if cancelled/preempted
      if (checkAndWarnIfCancelled(undocking_action_server_, "undock_robot") ||
        checkAndWarnIfPreempted(undocking_action_server_, "undock_robot"))
      {
        publishZeroVelocity();
        undocking_action_server_->terminate_all(result);
        return;
      }

      // Don't control the robot until charging is disabled
      if (!dock->disableCharging()) {
        loop_rate.sleep();
        continue;
      }

      // Get command to approach staging pose
      auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
      command->header.stamp = now();
      if (getCommandToPose(
          command->twist, staging_pose, undock_linear_tolerance_, undock_angular_tolerance_,
          !dock_backwards_))
      {
        RCLCPP_INFO(get_logger(), "Robot has reached staging pose");
        // Have reached staging_pose
        vel_publisher_->publish(std::move(command));
        if (dock->hasStoppedCharging()) {
          RCLCPP_INFO(get_logger(), "Robot has undocked!");
          result->success = true;
          curr_dock_type_.clear();
          publishZeroVelocity();
          undocking_action_server_->succeeded_current(result);
          return;
        }
        // Haven't stopped charging?
        throw opennav_docking_core::FailedToControl("Failed to control off dock, still charging");
      }

      // Publish command and sleep
      vel_publisher_->publish(std::move(command));
      loop_rate.sleep();
    }
  } catch (const tf2::TransformException & e) {
    result->error_msg = std::string("Transform error: ") + e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (opennav_docking_core::DockNotValid & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToControl & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::DockingException & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    result->error_msg = std::string("Internal error: ") + e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  publishZeroVelocity();
  undocking_action_server_->terminate_current(result);
}

geometry_msgs::msg::PoseStamped DockingServer::getRobotPoseInFrame(const std::string & frame)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame_;
  robot_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(robot_pose, robot_pose, frame);
  return robot_pose;
}

void DockingServer::publishZeroVelocity()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = now();
  vel_publisher_->publish(std::move(cmd_vel));
}

void DockingServer::publishDockingFeedback(uint16_t state)
{
  auto feedback = std::make_shared<DockRobot::Feedback>();
  feedback->state = state;
  feedback->docking_time = this->now() - action_start_time_;
  feedback->num_retries = num_retries_;
  docking_action_server_->publish_feedback(feedback);
}

rcl_interfaces::msg::SetParametersResult
DockingServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "initial_perception_timeout") {
        initial_perception_timeout_ = parameter.as_double();
      } else if (name == "wait_charge_timeout") {
        wait_charge_timeout_ = parameter.as_double();
      } else if (name == "undock_linear_tolerance") {
        undock_linear_tolerance_ = parameter.as_double();
      } else if (name == "undock_angular_tolerance") {
        undock_angular_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "base_frame") {
        base_frame_ = parameter.as_string();
      } else if (name == "fixed_frame") {
        fixed_frame_ = parameter.as_string();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "max_retries") {
        max_retries_ = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace opennav_docking

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(opennav_docking::DockingServer)
