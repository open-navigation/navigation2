// Copyright (c) 2021 Samsung Research
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

#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "nav2_bt_navigator/navigators/navigate_through_poses.hpp"
#include "nav2_core/navigator_exceptions.hpp"

namespace nav2_bt_navigator
{

bool
NavigateThroughPosesNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  if (!node->has_parameter("goals_blackboard_id")) {
    node->declare_parameter("goals_blackboard_id", std::string("goals"));
  }

  goals_blackboard_id_ = node->get_parameter("goals_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }

  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  // Odometry smoother object for getting current speed
  odom_smoother_ = odom_smoother;

  current_error_code_ = ActionT::Result::NONE;
  current_error_msg_ = "";
  return true;
}

std::string
NavigateThroughPosesNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter("default_nav_through_poses_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    node->declare_parameter<std::string>(
      "default_nav_through_poses_bt_xml",
      pkg_share_dir +
      "/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml");
  }

  node->get_parameter("default_nav_through_poses_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool
NavigateThroughPosesNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    current_error_code_ = ActionT::Result::FAILED_TO_LOAD_BEHAVIOR_TREE;
    current_error_msg_ = "Error loading XML file: " + bt_xml_filename +
      ". Navigation canceled.";
    RCLCPP_ERROR(logger_, current_error_msg_.c_str());
    return false;
  }

  return initializeGoalPoses(goal);
}

void
NavigateThroughPosesNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus /*final_bt_status*/)
{
  result->error_code = current_error_code_;
  result->error_msg = current_error_msg_;
}

void
NavigateThroughPosesNavigator::onLoop()
{
  using namespace nav2_util::geometry_utils;  // NOLINT

  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal, etc)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  auto blackboard = bt_action_server_->getBlackboard();

  Goals goal_poses;
  [[maybe_unused]] auto res = blackboard->get(goals_blackboard_id_, goal_poses);

  if (goal_poses.size() == 0) {
    bt_action_server_->publishFeedback(feedback_msg);
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *feedback_utils_.tf,
      feedback_utils_.global_frame, feedback_utils_.robot_frame,
      feedback_utils_.transform_tolerance))
  {
    throw nav2_core::NavigatorPoseNotAvailable("Robot pose is not available.");
  }

  try {
    // Get current path points
    nav_msgs::msg::Path current_path;
    if (!blackboard->get(path_blackboard_id_, current_path) || current_path.poses.size() == 0u) {
      // If no path set yet or not meaningful, can't compute ETA or dist remaining yet.
      throw nav2_core::NavigatorInvalidPath("no valid path available");
    }

    // Find the closest pose to current pose on global path
    auto find_closest_pose_idx =
      [&current_pose, &current_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
          double curr_dist = nav2_util::geometry_utils::euclidean_distance(
            current_pose, current_path.poses[curr_idx]);
          if (curr_dist < curr_min_dist) {
            curr_min_dist = curr_dist;
            closest_pose_idx = curr_idx;
          }
        }
        return closest_pose_idx;
      };

    // Calculate distance on the path
    double distance_remaining =
      nav2_util::geometry_utils::calculate_path_length(current_path, find_closest_pose_idx());

    // Default value for time remaining
    rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);

    // Get current speed
    geometry_msgs::msg::Twist current_odom = odom_smoother_->getTwist();
    double current_linear_speed = std::hypot(current_odom.linear.x, current_odom.linear.y);

    // Calculate estimated time taken to goal if speed is higher than 1cm/s
    // and at least 10cm to go
    if ((std::abs(current_linear_speed) > 0.01) && (distance_remaining > 0.1)) {
      estimated_time_remaining =
        rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
    }

    feedback_msg->distance_remaining = distance_remaining;
    feedback_msg->estimated_time_remaining = estimated_time_remaining;
  } catch (const nav2_core::NavigatorPoseNotAvailable & ex) {
    current_error_code_ = ActionT::Result::POSE_NOT_AVAILABLE;
    current_error_msg_ = ex.what();
    RCLCPP_ERROR(logger_, current_error_msg_.c_str());
    // Returning since no point attempting recovery or publishing
    // feedback until robot pose is available.
    return;
  } catch (const nav2_core::NavigatorInvalidPath & ex) {
    current_error_code_ = ActionT::Result::NO_VALID_PATH;
    current_error_msg_ = ex.what();
    // Ignore ??
  } catch (const std::runtime_error & ex) {
    current_error_code_ = ActionT::Result::UNKNOWN;
    current_error_msg_ = ex.what();
    // Ignore ??
  }

  int recovery_count = 0;
  res = blackboard->get("number_recoveries", recovery_count);
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->current_pose = current_pose;
  feedback_msg->navigation_time = clock_->now() - start_time_;
  feedback_msg->number_of_poses_remaining = goal_poses.size();

  bt_action_server_->publishFeedback(feedback_msg);
}

void
NavigateThroughPosesNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    if (!initializeGoalPoses(bt_action_server_->acceptPendingGoal())) {
      RCLCPP_WARN(
        logger_,
        "Preemption request was rejected since the goal poses could not be "
        "transformed. For now, continuing to track the last goal until completion.");
      bt_action_server_->terminatePendingGoal();
    }
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

bool
NavigateThroughPosesNavigator::initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal)
{
  Goals goal_poses = goal->poses;
  for (auto & goal_pose : goal_poses) {
    if (!nav2_util::transformPoseInTargetFrame(
        goal_pose, goal_pose, *feedback_utils_.tf, feedback_utils_.global_frame,
        feedback_utils_.transform_tolerance))
    {
      current_error_code_ = ActionT::Result::TF_ERROR;
      current_error_msg_ =
        "Failed to transform a goal pose provided with frame_id '" +
        goal_pose.header.frame_id +
        "' to the global frame '" +
        feedback_utils_.global_frame +
        "'.";

      RCLCPP_ERROR(logger_, current_error_msg_.c_str());
      return false;
    }
  }

  if (goal_poses.size() > 0) {
    RCLCPP_INFO(
      logger_, "Begin navigating from current location through %zu poses to (%.2f, %.2f)",
      goal_poses.size(), goal_poses.back().pose.position.x, goal_poses.back().pose.position.y);
  }

  // Reset state for new action feedback
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set("number_recoveries", 0);  // NOLINT

  // Update the goal pose on the blackboard
  blackboard->set<Goals>(goals_blackboard_id_, std::move(goal_poses));

  return true;
}

}  // namespace nav2_bt_navigator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_bt_navigator::NavigateThroughPosesNavigator,
  nav2_core::NavigatorBase)
