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

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>
#include <set>

namespace nav2_bt_navigator
{

BtNavigatorBase::BtNavigatorBase()
: nav2_util::LifecycleNode("bt_navigator", "", false)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_distance_traveled_condition_bt_node"
  };

  // Declare this node's parameters
  declare_parameter("bt_xml_filename");
  declare_parameter("plugin_lib_names", plugin_libs);
}

BtNavigatorBase::~BtNavigatorBase()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
BtNavigatorBase::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // use suffix '_rclcpp_node' to keep parameter file consistency #1773
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r", std::string("__node:=") + get_name() + "_rclcpp_node",
      "--"});
  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  // Get the libraries to pull plugins from
  plugin_lib_names_ = get_parameter("plugin_lib_names").as_string_array();

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));  // NOLINT
  blackboard_->set<bool>("path_updated", false);  // NOLINT
  blackboard_->set<bool>("initial_pose_received", false);  // NOLINT
  blackboard_->set<int>("number_recoveries", 0);  // NOLINT

  // Get the BT filename to use from the node parameter
  std::string bt_xml_filename;
  get_parameter("bt_xml_filename", bt_xml_filename);

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }

  xml_string_ = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

  RCLCPP_DEBUG(get_logger(), "Behavior Tree file: '%s'", bt_xml_filename.c_str());
  RCLCPP_DEBUG(get_logger(), "Behavior Tree XML: %s", xml_string_.c_str());

  // Create the Behavior Tree from the XML input
  tree_ = bt_->buildTreeFromText(xml_string_, blackboard_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigatorBase::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigatorBase::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigatorBase::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // TODO(orduno) Fix the race condition between the worker thread ticking the tree
  //              and the main thread resetting the resources, see #1344
  client_node_.reset();

  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

  plugin_lib_names_.clear();
  xml_string_.clear();
  blackboard_.reset();
  bt_->haltAllActions(tree_.rootNode());
  bt_.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigatorBase::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigatorBase::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace nav2_bt_navigator
