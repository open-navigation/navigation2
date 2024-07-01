// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
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
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_planner/planner_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("planner_server", "", options),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_ids_{"GridBased"},
  default_types_{"nav2_navfn_planner::NavfnPlanner"},
  costmap_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);
  declare_parameter("action_server_result_timeout", 10.0);

  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap",
    get_parameter("use_sim_time").as_bool());
}

PlannerServer::~PlannerServer()
{
  /*
   * Backstop ensuring this state is destroyed, even if deactivate/cleanup are
   * never called.
   */
  planners_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->configure();
  costmap_ = costmap_ros_->getCostmap();

  if (!costmap_ros_->getUseRadius()) {
    collision_checker_ =
      std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
      costmap_);
  }

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  RCLCPP_DEBUG(
    get_logger(), "Costmap size: %d,%d",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = nav2_util::get_plugin_type_param(
        node, planner_ids_[i]);
      nav2_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created global planner plugin %s of type %s",
        planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->configure(node, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create global planner. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  double action_server_result_timeout;
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  // Create the action servers for path planning to a pose and through poses
  action_server_pose_ = std::make_unique<ActionServerToPose>(
    shared_from_this(),
    "compute_path_to_pose",
    std::bind(&PlannerServer::computePlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true, server_options);

  action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
    shared_from_this(),
    "compute_path_through_poses",
    std::bind(&PlannerServer::computePlanThroughPoses, this),
    nullptr,
    std::chrono::milliseconds(500),
    true, server_options);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_pose_->activate();
  action_server_poses_->activate();
  const auto costmap_ros_state = costmap_ros_->activate();
  if (costmap_ros_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  auto node = shared_from_this();

  is_path_valid_service_ = node->create_service<nav2_msgs::srv::IsPathValid>(
    "is_path_valid",
    std::bind(
      &PlannerServer::isPathValid, this,
      std::placeholders::_1, std::placeholders::_2));

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PlannerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_pose_->deactivate();
  action_server_poses_->deactivate();
  plan_publisher_->on_deactivate();

  /*
   * The costmap is also a lifecycle node, so it may have already fired on_deactivate
   * via rcl preshutdown cb. Despite the rclcpp docs saying on_shutdown callbacks fire
   * in the order added, the preshutdown callbacks clearly don't per se, due to using an
   * unordered_set iteration. Once this issue is resolved, we can maybe make a stronger
   * ordering assumption: https://github.com/ros2/rclcpp/issues/2096
   */
  costmap_ros_->deactivate();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_pose_.reset();
  action_server_poses_.reset();
  plan_publisher_.reset();
  tf_.reset();

  costmap_ros_->cleanup();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }

  planners_.clear();
  costmap_thread_.reset();
  costmap_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void PlannerServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool PlannerServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
bool PlannerServer::getStartPose(
  typename std::shared_ptr<const typename T::Goal> goal,
  geometry_msgs::msg::PoseStamped & start)
{
  if (goal->use_start) {
    start = goal->start;
  } else if (!costmap_ros_->getRobotPose(start)) {
    return false;
  }

  return true;
}

bool PlannerServer::transformPosesToGlobalFrame(
  geometry_msgs::msg::PoseStamped & curr_start,
  geometry_msgs::msg::PoseStamped & curr_goal)
{
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal))
  {
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::validatePath(
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & path,
  const std::string & planner_id)
{
  if (path.poses.empty()) {
    RCLCPP_WARN(
      get_logger(), "Planning algorithm %s failed to generate a valid"
      " path to (%.2f, %.2f)", planner_id.c_str(),
      goal.pose.position.x, goal.pose.position.y);
    return false;
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Found valid path of size %zu to (%.2f, %.2f)",
    path.poses.size(), goal.pose.position.x,
    goal.pose.position.y);

  return true;
}

void PlannerServer::computePlanThroughPoses()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = this->now();

  // Initialize the ComputePathThroughPoses goal and result
  auto goal = action_server_poses_->get_current_goal();
  auto result = std::make_shared<ActionThroughPoses::Result>();
  nav_msgs::msg::Path concat_path;

  geometry_msgs::msg::PoseStamped curr_start, curr_goal;

  try {
    if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_poses_, goal);

    if (goal->goals.empty()) {
      throw nav2_core::NoViapointsGiven("No viapoints given");
    }

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose<ActionThroughPoses>(goal, start)) {
      throw nav2_core::PlannerTFError("Unable to get start pose");
    }

    auto cancel_checker = [this]() {
        return action_server_poses_->is_cancel_requested();
      };

    // Get consecutive paths through these points
    for (unsigned int i = 0; i != goal->goals.size(); i++) {
      // Get starting point
      if (i == 0) {
        curr_start = start;
      } else {
        // pick the end of the last planning task as the start for the next one
        // to allow for path tolerance deviations
        curr_start = concat_path.poses.back();
        curr_start.header = concat_path.header;
      }
      curr_goal = goal->goals[i];

      // Transform them into the global frame
      if (!transformPosesToGlobalFrame(curr_start, curr_goal)) {
        throw nav2_core::PlannerTFError("Unable to transform poses to global frame");
      }

      // Get plan from start -> goal
      nav_msgs::msg::Path curr_path = getPlan(
        curr_start, curr_goal, goal->planner_id,
        cancel_checker);

      if (!validatePath<ActionThroughPoses>(curr_goal, curr_path, goal->planner_id)) {
        throw nav2_core::NoValidPathCouldBeFound(goal->planner_id + " generated a empty path");
      }

      // Concatenate paths together
      concat_path.poses.insert(
        concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
      concat_path.header = curr_path.header;
    }

    // Publish the plan for visualization purposes
    result->path = concat_path;
    publishPlan(result->path);

    auto cycle_duration = this->now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_poses_->succeeded_current(result);
  } catch (nav2_core::InvalidPlanner & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionThroughPosesResult::INVALID_PLANNER;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::StartOccupied & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionThroughPosesResult::START_OCCUPIED;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::GoalOccupied & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionThroughPosesResult::GOAL_OCCUPIED;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::NoValidPathCouldBeFound & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionThroughPosesResult::NO_VALID_PATH;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::PlannerTimedOut & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionThroughPosesResult::TIMEOUT;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::StartOutsideMapBounds & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionThroughPosesResult::START_OUTSIDE_MAP;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::GoalOutsideMapBounds & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionThroughPosesResult::GOAL_OUTSIDE_MAP;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::PlannerTFError & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionThroughPosesResult::TF_ERROR;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::NoViapointsGiven & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionThroughPosesResult::NO_VIAPOINTS_GIVEN;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::PlannerCancelled &) {
    result->error_msg = "Goal was canceled. Canceling planning action.";
    RCLCPP_INFO(get_logger(), result->error_msg.c_str());
    action_server_poses_->terminate_all();
  } catch (std::exception & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionThroughPosesResult::UNKNOWN;
    action_server_poses_->terminate_current(result);
  }
}

void
PlannerServer::computePlan()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = this->now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();

  geometry_msgs::msg::PoseStamped start;

  try {
    if (isServerInactive(action_server_pose_) || isCancelRequested(action_server_pose_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_pose_, goal);

    // Use start pose if provided otherwise use current robot pose
    if (!getStartPose<ActionToPose>(goal, start)) {
      throw nav2_core::PlannerTFError("Unable to get start pose");
    }

    // Transform them into the global frame
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    if (!transformPosesToGlobalFrame(start, goal_pose)) {
      throw nav2_core::PlannerTFError("Unable to transform poses to global frame");
    }

    auto cancel_checker = [this]() {
        return action_server_pose_->is_cancel_requested();
      };

    result->path = getPlan(start, goal_pose, goal->planner_id, cancel_checker);

    if (!validatePath<ActionThroughPoses>(goal_pose, result->path, goal->planner_id)) {
      throw nav2_core::NoValidPathCouldBeFound(goal->planner_id + " generated a empty path");
    }

    // Publish the plan for visualization purposes
    publishPlan(result->path);

    auto cycle_duration = this->now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }
    action_server_pose_->succeeded_current(result);
  } catch (nav2_core::InvalidPlanner & ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionToPoseResult::INVALID_PLANNER;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::StartOccupied & ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionToPoseResult::START_OCCUPIED;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::GoalOccupied & ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionToPoseResult::GOAL_OCCUPIED;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::NoValidPathCouldBeFound & ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionToPoseResult::NO_VALID_PATH;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::PlannerTimedOut & ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionToPoseResult::TIMEOUT;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::StartOutsideMapBounds & ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionToPoseResult::START_OUTSIDE_MAP;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::GoalOutsideMapBounds & ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionToPoseResult::GOAL_OUTSIDE_MAP;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::PlannerTFError & ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionToPoseResult::TF_ERROR;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::PlannerCancelled &) {
    result->error_msg = "Goal was canceled. Canceling planning action.";
    RCLCPP_INFO(get_logger(), result->error_msg.c_str());
    action_server_pose_->terminate_all();
  } catch (std::exception & ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex, result->error_msg);
    result->error_code = ActionToPoseResult::UNKNOWN;
    action_server_pose_->terminate_current(result);
  }
}

nav_msgs::msg::Path
PlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id,
  std::function<bool()> cancel_checker)
{
  RCLCPP_DEBUG(
    get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  if (planners_.find(planner_id) != planners_.end()) {
    return planners_[planner_id]->createPlan(start, goal, cancel_checker);
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No planners specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goal, cancel_checker);
    } else {
      RCLCPP_ERROR(
        get_logger(), "planner %s is not a valid planner. "
        "Planner names are: %s", planner_id.c_str(),
        planner_ids_concat_.c_str());
      throw nav2_core::InvalidPlanner("Planner id " + planner_id + " is invalid");
    }
  }

  return nav_msgs::msg::Path();
}

void
PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

void PlannerServer::isPathValid(
  const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
  std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
{
  response->is_valid = true;

  if (request->path.poses.empty()) {
    response->is_valid = false;
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  unsigned int closest_point_index = 0;
  if (costmap_ros_->getRobotPose(current_pose)) {
    float current_distance = std::numeric_limits<float>::max();
    float closest_distance = current_distance;
    geometry_msgs::msg::Point current_point = current_pose.pose.position;
    for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
      geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

      current_distance = nav2_util::geometry_utils::euclidean_distance(
        current_point,
        path_point);

      if (current_distance < closest_distance) {
        closest_point_index = i;
        closest_distance = current_distance;
      }
    }

    /**
     * The lethal check starts at the closest point to avoid points that have already been passed
     * and may have become occupied. The method for collision detection is based on the shape of
     * the footprint.
     */
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    unsigned int mx = 0;
    unsigned int my = 0;

    bool use_radius = costmap_ros_->getUseRadius();

    unsigned int cost = nav2_costmap_2d::FREE_SPACE;
    for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
      auto & position = request->path.poses[i].pose.position;
      if (use_radius) {
        if (costmap_->worldToMap(position.x, position.y, mx, my)) {
          cost = costmap_->getCost(mx, my);
        } else {
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        }
      } else {
        nav2_costmap_2d::Footprint footprint = costmap_ros_->getRobotFootprint();
        auto theta = tf2::getYaw(request->path.poses[i].pose.orientation);
        cost = static_cast<unsigned int>(collision_checker_->footprintCostAtPose(
            position.x, position.y, theta, footprint));
      }

      if (use_radius &&
        (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
      {
        response->is_valid = false;
        break;
      } else if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        response->is_valid = false;
        break;
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult
PlannerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          max_planner_duration_ = 1 / parameter.as_double();
        } else {
          RCLCPP_WARN(
            get_logger(),
            "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
            " than 0.0 to turn on duration overrrun warning messages", parameter.as_double());
          max_planner_duration_ = 0.0;
        }
      }
    }
  }

  result.successful = true;
  return result;
}

void PlannerServer::exceptionWarning(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id,
  const std::exception & ex,
  std::string & error_msg)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2)
     << planner_id << "plugin failed to plan from ("
     << start.pose.position.x << ", " << start.pose.position.y
     << ") to ("
     << goal.pose.position.x << ", " << goal.pose.position.y << ")"
     << ": \"" << ex.what() << "\"";

  error_msg = ss.str();
  RCLCPP_WARN(get_logger(), error_msg.c_str());
}

}  // namespace nav2_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_planner::PlannerServer)
