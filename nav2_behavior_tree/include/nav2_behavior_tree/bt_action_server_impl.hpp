// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_

#include <memory>
#include <string>
#include <fstream>
#include <set>
#include <exception>
#include <vector>
#include <limits>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behavior_tree
{

template<class ActionT>
BtActionServer<ActionT>::BtActionServer(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & action_name,
  const std::vector<std::string> & plugin_lib_names,
  const std::string & default_bt_xml_filename,
  OnGoalReceivedCallback on_goal_received_callback,
  OnLoopCallback on_loop_callback,
  OnPreemptCallback on_preempt_callback,
  OnCompletionCallback on_completion_callback)
: action_name_(action_name),
  default_bt_xml_filename_(default_bt_xml_filename),
  plugin_lib_names_(plugin_lib_names),
  node_(parent),
  on_goal_received_callback_(on_goal_received_callback),
  on_loop_callback_(on_loop_callback),
  on_preempt_callback_(on_preempt_callback),
  on_completion_callback_(on_completion_callback)
{
  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare this node's parameters
  if (!node->has_parameter("bt_loop_duration")) {
    node->declare_parameter("bt_loop_duration", 10);
  }
  if (!node->has_parameter("default_server_timeout")) {
    node->declare_parameter("default_server_timeout", 20);
  }
  if (!node->has_parameter("action_server_result_timeout")) {
    node->declare_parameter("action_server_result_timeout", 900.0);
  }
  if (!node->has_parameter("always_reload_bt_xml")) {
    node->declare_parameter("always_reload_bt_xml", false);
  }
  if (!node->has_parameter("wait_for_service_timeout")) {
    node->declare_parameter("wait_for_service_timeout", 1000);
  }

  std::vector<std::string> error_code_names = {
    "follow_path_error_code",
    "compute_path_error_code"
  };

  if (!node->has_parameter("error_code_names")) {
    const rclcpp::ParameterValue value = node->declare_parameter(
      "error_code_names",
      rclcpp::PARAMETER_STRING_ARRAY);
    if (value.get_type() == rclcpp::PARAMETER_NOT_SET) {
      std::string error_codes_str;
      for (const auto & error_code : error_code_names) {
        error_codes_str += " " + error_code;
      }
      RCLCPP_WARN_STREAM(
        logger_, "Error_code parameters were not set. Using default values of:"
          << error_codes_str + "\n"
          << "Make sure these match your BT and there are not other sources of error codes you"
          "reported to your application");
      rclcpp::Parameter error_code_names_param("error_code_names", error_code_names);
      node->set_parameter(error_code_names_param);
    } else {
      error_code_names = value.get<std::vector<std::string>>();
      std::string error_codes_str;
      for (const auto & error_code : error_code_names) {
        error_codes_str += " " + error_code;
      }
      RCLCPP_INFO_STREAM(logger_, "Error_code parameters were set to:" << error_codes_str);
    }
  }
}

template<class ActionT>
BtActionServer<ActionT>::~BtActionServer()
{}

template<class ActionT>
bool BtActionServer<ActionT>::on_configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Name client node after action name
  std::string client_node_name = action_name_;
  std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
  // Use suffix '_rclcpp_node' to keep parameter file consistency #1773
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r",
      std::string("__node:=") +
      std::string(node->get_name()) + "_" + client_node_name + "_rclcpp_node",
      "-p",
      "use_sim_time:=" +
      std::string(node->get_parameter("use_sim_time").as_bool() ? "true" : "false"),
      "--"});

  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  // Declare parameters for common client node applications to share with BT nodes
  // Declare if not declared in case being used an external application, then copying
  // all of the main node's parameters to the client for BT nodes to obtain
  nav2_util::declare_parameter_if_not_declared(
    node, "global_frame", rclcpp::ParameterValue(std::string("map")));
  nav2_util::declare_parameter_if_not_declared(
    node, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  nav2_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  rclcpp::copy_all_parameter_values(node, client_node_);

  // set the timeout in seconds for the action server to discard goal handles if not finished
  double action_server_result_timeout =
    node->get_parameter("action_server_result_timeout").as_double();
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  action_server_ = std::make_shared<ActionServer>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    action_name_, std::bind(&BtActionServer<ActionT>::executeCallback, this),
    nullptr, std::chrono::milliseconds(500), false, server_options);

  // Get parameters for BT timeouts
  int bt_loop_duration;
  node->get_parameter("bt_loop_duration", bt_loop_duration);
  bt_loop_duration_ = std::chrono::milliseconds(bt_loop_duration);
  int default_server_timeout;
  node->get_parameter("default_server_timeout", default_server_timeout);
  default_server_timeout_ = std::chrono::milliseconds(default_server_timeout);
  int wait_for_service_timeout;
  node->get_parameter("wait_for_service_timeout", wait_for_service_timeout);
  wait_for_service_timeout_ = std::chrono::milliseconds(wait_for_service_timeout);
  node->get_parameter("always_reload_bt_xml", always_reload_bt_xml_);

  // Get error code id names to grab off of the blackboard
  error_code_names_ = node->get_parameter("error_code_names").as_string_array();

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_, client_node_);

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>(
    "wait_for_service_timeout",
    wait_for_service_timeout_);

  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_activate()
{
  if (!loadBehaviorTree(default_bt_xml_filename_)) {
    RCLCPP_ERROR(logger_, "Error loading XML file: %s", default_bt_xml_filename_.c_str());
    return false;
  }
  action_server_->activate();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_deactivate()
{
  action_server_->deactivate();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_cleanup()
{
  client_node_.reset();
  action_server_.reset();
  topic_logger_.reset();
  plugin_lib_names_.clear();
  current_bt_xml_filename_.clear();
  blackboard_.reset();
  bt_->haltAllActions(tree_);
  bt_.reset();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // Empty filename is default for backward compatibility
  auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

  // Use previous BT if it is the existing one and always reload flag is not set to true
  if (!always_reload_bt_xml_ && current_bt_xml_filename_ == filename) {
    RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
    return true;
  }

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", filename.c_str());
    return false;
  }

  // Create the Behavior Tree from the XML input
  try {
    tree_ = bt_->createTreeFromFile(filename, blackboard_);
    for (auto & subtree : tree_.subtrees) {
      auto & blackboard = subtree->blackboard;
      blackboard->set("node", client_node_);
      blackboard->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
      blackboard->set<std::chrono::milliseconds>(
        "wait_for_service_timeout",
        wait_for_service_timeout_);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception when loading BT: %s", e.what());
    return false;
  }

  topic_logger_ = std::make_unique<RosTopicLogger>(client_node_, tree_);

  current_bt_xml_filename_ = filename;
  return true;
}

template<class ActionT>
void BtActionServer<ActionT>::executeCallback()
{
  if (!on_goal_received_callback_(action_server_->get_current_goal())) {
    // Give server an opportunity to populate the result message
    auto result = std::make_shared<typename ActionT::Result>();
    populateErrorCode(result);
    action_server_->terminate_current(result);
    cleanErrorCodes();
    return;
  }

  auto is_canceling = [&]() {
      if (action_server_ == nullptr) {
        RCLCPP_DEBUG(logger_, "Action server unavailable. Canceling.");
        return true;
      }
      if (!action_server_->is_server_active()) {
        RCLCPP_DEBUG(logger_, "Action server is inactive. Canceling.");
        return true;
      }
      return action_server_->is_cancel_requested();
    };

  auto on_loop = [&]() {
      if (action_server_->is_preempt_requested() && on_preempt_callback_) {
        on_preempt_callback_(action_server_->get_pending_goal());
      }
      topic_logger_->flush();
      on_loop_callback_();
    };

  // Execute the BT that was previously created in the configure step
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);

  // Make sure that the Bt is not in a running state from a previous execution
  // note: if all the ControlNodes are implemented correctly, this is not needed.
  bt_->haltAllActions(tree_);

  // Give server an opportunity to populate the result message or simple give
  // an indication that the action is complete.
  auto result = std::make_shared<typename ActionT::Result>();

  populateErrorCode(result);

  on_completion_callback_(result, rc);

  switch (rc) {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(logger_, "Goal succeeded");
      action_server_->succeeded_current(result);
      break;

    case nav2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(logger_, "Goal failed");
      action_server_->terminate_current(result);
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(logger_, "Goal canceled");
      action_server_->terminate_all(result);
      break;
  }

  cleanErrorCodes();
}

template<class ActionT>
void BtActionServer<ActionT>::populateErrorCode(
  typename std::shared_ptr<typename ActionT::Result> result)
{
  int highest_priority_error_code = std::numeric_limits<int>::max();
  std::string highest_priority_error_msg = "";
  for (const auto & error_code : error_code_names_) {
    try {
      int current_error_code = blackboard_->get<int>(error_code);
      if (current_error_code != 0 && current_error_code < highest_priority_error_code) {
        highest_priority_error_code = current_error_code;
      }
    } catch (...) {
      RCLCPP_DEBUG(
        logger_,
        "Failed to get error code: %s from blackboard",
        error_code.c_str());
    }
  }

  if (highest_priority_error_code != std::numeric_limits<int>::max()) {
    result->error_code = highest_priority_error_code;
    result->error_msg = highest_priority_error_msg;
  }
}

template<class ActionT>
void BtActionServer<ActionT>::cleanErrorCodes()
{
  for (const auto & error_code : error_code_names_) {
    blackboard_->set<unsigned short>(error_code, 0);  //NOLINT
  }
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
