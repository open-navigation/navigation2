// Copyright (c) 2018 Intel Corporation
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

#include "nav2_bt_navigator/bt_navigator.hpp"

#include <memory>
#include <string>
#include <utility>
#include <set>
#include <limits>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("bt_navigator", "", options),
  navigator_class_loader_("nav2_bt_navigator", "nav2_bt_navigator::NavigatorBase")
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
    "nav2_smooth_path_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_drive_on_heading_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_globally_updated_goal_condition_bt_node",
    "nav2_is_path_valid_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_truncate_path_local_action_bt_node",
    "nav2_goal_updater_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_path_expiring_timer_condition",
    "nav2_distance_traveled_condition_bt_node",
    "nav2_single_trigger_bt_node",
    "nav2_is_battery_low_condition_bt_node",
    "nav2_navigate_through_poses_action_bt_node",
    "nav2_navigate_to_pose_action_bt_node",
    "nav2_remove_passed_goals_action_bt_node",
    "nav2_planner_selector_bt_node",
    "nav2_controller_selector_bt_node",
    "nav2_goal_checker_selector_bt_node",
    "nav2_controller_cancel_bt_node",
    "nav2_path_longer_on_approach_bt_node"
    "nav2_wait_cancel_bt_node",
    "nav2_spin_cancel_bt_node",
    "nav2_back_up_cancel_bt_node"
    "nav2_drive_on_heading_cancel_bt_node"
  };

  const std::vector<std::string> navigators = {
    "navigate_to_pose",
    "navigate_through_poses"
  };

  declare_parameter("navigator", navigators);
  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter("global_frame", std::string("map"));
  declare_parameter("robot_base_frame", std::string("base_link"));
  declare_parameter("odom_topic", std::string("odom"));

}

BtNavigator::~BtNavigator()
{
}

nav2_util::CallbackReturn
BtNavigator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_base_frame").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  odom_topic_ = get_parameter("odom_topic").as_string();

  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  // Odometry smoother object for getting current speed
  odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(shared_from_this(), 0.3, odom_topic_);

  if(!loadNavigatorPlugins(plugin_lib_names)){
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // activate all plugin
  for(size_t i = 0; i < navigators_.size(); i++){
    if(!navigators_[i]->on_activate()){
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  for(size_t i = 0; i < navigators_.size(); i++){
    if(!navigators_[i]->on_deactivate()){
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

  for(size_t i = 0; i < navigators_.size(); i++){
    if(!navigators_[i]->on_cleanup()){
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  navigators_.clear();
  navigator_ids_.clear();
  navigator_types_.clear();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool 
BtNavigator::loadNavigatorPlugins(std::vector<std::string> plugin_names)
{
  get_parameter("navigator", navigator_ids_);

  nav2_core::FeedbackUtils feedback_utils;
  feedback_utils.tf = tf_;
  feedback_utils.global_frame = global_frame_;
  feedback_utils.robot_frame = robot_frame_;
  feedback_utils.transform_tolerance = transform_tolerance_;

  for(std::string navigator_id: navigator_ids_){
    std::string navigator_type = nav2_util::get_plugin_type_param(shared_from_this(), navigator_id);
    navigators_.push_back(navigator_class_loader_.createUniqueInstance(navigator_type));
    navigator_types_.push_back(navigator_type);
    if(!navigators_.back()->on_configure(weak_from_this(), plugin_names, feedback_utils, &plugin_muxer_, odom_smoother_)){
      return false;
    }
  }

  return true;
}

}  // namespace nav2_bt_navigator

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_bt_navigator::BtNavigator)
