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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_behavior_tree/ros_topic_logger.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_behavior_tree
{
/**
 * @class nav2_behavior_tree::BtActionServer
 * @brief An action server that uses behavior tree to execute an action
 */
template<class ActionT>
class BtActionServer
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  typedef std::function<bool (typename ActionT::Goal::ConstSharedPtr)> OnGoalReceivedCallback;
  typedef std::function<void ()> OnLoopCallback;
  typedef std::function<void (typename ActionT::Goal::ConstSharedPtr)> OnPreemptCallback;
  typedef std::function<void (typename ActionT::Result::SharedPtr,
      nav2_behavior_tree::BtStatus)> OnCompletionCallback;

  /**
   * @brief A constructor for nav2_behavior_tree::BtActionServer class
   */
  explicit BtActionServer(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & action_name,
    const std::vector<std::string> & plugin_lib_names,
    const std::string & default_bt_xml_filename,
    OnGoalReceivedCallback on_goal_received_callback,
    OnLoopCallback on_loop_callback,
    OnPreemptCallback on_preempt_callback,
    OnCompletionCallback on_completion_callback);

  /**
   * @brief A destructor for nav2_behavior_tree::BtActionServer class
   */
  ~BtActionServer();

  /**
   * @brief Configures member variables
   * Initializes action server for, builds behavior tree from xml file,
   * and calls user-defined onConfigure.
   * @return bool true on SUCCESS and false on FAILURE
   */
  bool on_configure();

  /**
   * @brief Activates action server
   * @return bool true on SUCCESS and false on FAILURE
   */
  bool on_activate();

  /**
   * @brief Deactivates action server
   * @return bool true on SUCCESS and false on FAILURE
   */
  bool on_deactivate();

  /**
   * @brief Resets member variables
   * @return bool true on SUCCESS and false on FAILURE
   */
  bool on_cleanup();

  /**
   * @brief Replace current BT with another one
   * @param bt_xml_filename The file containing the new BT, uses default filename if empty
   * @return bool true if the resulting BT correspond to the one in bt_xml_filename. false
   * if something went wrong, and previous BT is maintained
   */
  bool loadBehaviorTree(const std::string & bt_xml_filename = "");

  /**
   * @brief Getter function for BT Blackboard
   * @return BT::Blackboard::Ptr Shared pointer to current BT blackboard
   */
  BT::Blackboard::Ptr getBlackboard() const
  {
    return blackboard_;
  }

  /**
   * @brief Getter function for current BT XML filename
   * @return string Containing current BT XML filename
   */
  std::string getCurrentBTFilename() const
  {
    return current_bt_xml_filename_;
  }

  /**
   * @brief Getter function for default BT XML filename
   * @return string Containing default BT XML filename
   */
  std::string getDefaultBTFilename() const
  {
    return default_bt_xml_filename_;
  }

  /**
   * @brief Wrapper function to accept pending goal if a preempt has been requested
   * @return Shared pointer to pending action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> acceptPendingGoal()
  {
    return action_server_->accept_pending_goal();
  }

  /**
   * @brief Wrapper function to terminate pending goal if a preempt has been requested
   */
  void terminatePendingGoal()
  {
    action_server_->terminate_pending_goal();
  }

  /**
   * @brief Wrapper function to get current goal
   * @return Shared pointer to current action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
  {
    return action_server_->get_current_goal();
  }

  /**
   * @brief Wrapper function to get pending goal
   * @return Shared pointer to pending action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> getPendingGoal() const
  {
    return action_server_->get_pending_goal();
  }

  /**
   * @brief Wrapper function to publish action feedback
   */
  void publishFeedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
  {
    action_server_->publish_feedback(feedback);
  }

  /**
   * @brief Getter function for the current BT tree
   * @return BT::Tree Current behavior tree
   */
  const BT::Tree & getTree() const
  {
    return tree_;
  }

  /**
   * @brief Function to halt the current tree. It will interrupt the execution of RUNNING nodes
   * by calling their halt() implementation (only for Async nodes that may return RUNNING)
   * This should already done for all the exit states of the action but preemption
   */
  void haltTree()
  {
    tree_.haltTree();
  }

protected:
  /**
   * @brief Action server callback
   */
  void executeCallback();

  /**
   * @brief updates the action server result to the highest priority error code posted on the
   * blackboard
   * @param result the action server result to be updated
   */
  void populateErrorCode(typename std::shared_ptr<typename ActionT::Result> result);

  /**
   * @brief Setting BT error codes to success. Used to clean blackboard between different BT runs
   */
  void cleanErrorCodes();

  // Action name
  std::string action_name_;

  // Our action server implements the template action
  std::shared_ptr<ActionServer> action_server_;

  // Behavior Tree to be executed when goal is received
  BT::Tree tree_;

  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;

  // The XML file that cointains the Behavior Tree to create
  std::string current_bt_xml_filename_;
  std::string default_bt_xml_filename_;

  // The wrapper class for the BT functionality
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

  // Libraries to pull plugins (BT Nodes) from
  std::vector<std::string> plugin_lib_names_;

  // Error code id names
  std::vector<std::string> error_code_names_;

  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;

  // Parent node
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("BtActionServer")};

  // To publish BT logs
  std::unique_ptr<RosTopicLogger> topic_logger_;

  // Duration for each iteration of BT execution
  std::chrono::milliseconds bt_loop_duration_;

  // Default timeout value while waiting for response from a server
  std::chrono::milliseconds default_server_timeout_;

  // The timeout value for waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;

  // should the BT be reloaded even if the same xml filename is requested?
  bool always_reload_bt_xml_ = false;

  // User-provided callbacks
  OnGoalReceivedCallback on_goal_received_callback_;
  OnLoopCallback on_loop_callback_;
  OnPreemptCallback on_preempt_callback_;
  OnCompletionCallback on_completion_callback_;
};

}  // namespace nav2_behavior_tree

#include <nav2_behavior_tree/bt_action_server_impl.hpp>  // NOLINT(build/include_order)
#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_
