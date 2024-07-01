// Copyright (c) 2021-2023 Samsung Research America
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

#ifndef NAV2_CORE__BEHAVIOR_TREE_NAVIGATOR_HPP_
#define NAV2_CORE__BEHAVIOR_TREE_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"

namespace nav2_core
{

/**
 * @struct FeedbackUtils
 * @brief Navigator feedback utilities required to get transforms and reference frames.
 */
struct FeedbackUtils
{
  std::string robot_frame;
  std::string global_frame;
  double transform_tolerance;
  std::shared_ptr<tf2_ros::Buffer> tf;
};

/**
 * @class NavigatorMuxer
 * @brief A class to control the state of the BT navigator by allowing only a single
 * plugin to be processed at a time.
 */
class NavigatorMuxer
{
public:
  /**
   * @brief A Navigator Muxer constructor
   */
  NavigatorMuxer()
  : current_navigator_(std::string("")) {}

  /**
   * @brief Get the navigator muxer state
   * @return bool If a navigator is in progress
   */
  bool isNavigating()
  {
    std::scoped_lock l(mutex_);
    return !current_navigator_.empty();
  }

  /**
   * @brief Start navigating with a given navigator
   * @param string Name of the navigator to start
   */
  void startNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    if (!current_navigator_.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "Major error! Navigation requested while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin.");
    }
    current_navigator_ = navigator_name;
  }

  /**
   * @brief Stop navigating with a given navigator
   * @param string Name of the navigator ending task
   */
  void stopNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    if (current_navigator_ != navigator_name) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "Major error! Navigation stopped while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin.");
    } else {
      current_navigator_ = std::string("");
    }
  }

protected:
  std::string current_navigator_;
  std::mutex mutex_;
};

/**
 * @class NavigatorBase
 * @brief Navigator interface to allow navigators to be stored in a vector and
 * accessed via pluginlib due to templates. These functions will be implemented
 * by BehaviorTreeNavigator, not the user. The user should implement the virtual
 * methods from BehaviorTreeNavigator to implement their navigator action.
 */
class NavigatorBase
{
public:
  NavigatorBase() = default;
  virtual ~NavigatorBase() = default;

  /**
   * @brief Configuration of the navigator's backend BT and actions
   * @return bool If successful
   */
  virtual bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    const std::vector<std::string> & plugin_lib_names,
    const FeedbackUtils & feedback_utils,
    nav2_core::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) = 0;

  /**
   * @brief Activation of the navigator's backend BT and actions
   * @return bool If successful
   */
  virtual bool on_activate() = 0;

  /**
   * @brief Deactivation of the navigator's backend BT and actions
   * @return bool If successful
   */
  virtual bool on_deactivate() = 0;

  /**
   * @brief Cleanup a navigator
   * @return bool If successful
   */
  virtual bool on_cleanup() = 0;
};

/**
 * @class BehaviorTreeNavigator
 * @brief Navigator interface that acts as a base class for all BT-based Navigator action's plugins
 * All methods from NavigatorBase are marked as final so they may not be overrided by derived
 * methods - instead, users should use the appropriate APIs provided after BT Action handling.
 */
template<class ActionT>
class BehaviorTreeNavigator : public NavigatorBase
{
public:
  using Ptr = std::shared_ptr<nav2_core::BehaviorTreeNavigator<ActionT>>;

  /**
   * @brief A Navigator constructor
   */
  BehaviorTreeNavigator()
  : NavigatorBase()
  {
    plugin_muxer_ = nullptr;
  }

  /**
   * @brief Virtual destructor
   */
  virtual ~BehaviorTreeNavigator() = default;

  /**
   * @brief Configuration to setup the navigator's backend BT and actions
   * @param parent_node The ROS parent node to utilize
   * @param plugin_lib_names a vector of plugin shared libraries to load
   * @param feedback_utils Some utilities useful for navigators to have
   * @param plugin_muxer The muxing object to ensure only one navigator
   * can be active at a time
   * @param odom_smoother Object to get current smoothed robot's speed
   * @return bool If successful
   */
  bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    const std::vector<std::string> & plugin_lib_names,
    const FeedbackUtils & feedback_utils,
    nav2_core::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) final
  {
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    feedback_utils_ = feedback_utils;
    plugin_muxer_ = plugin_muxer;

    // get the default behavior tree for this navigator
    std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);

    // Create the Behavior Tree Action Server for this navigator
    bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
      node,
      getName(),
      plugin_lib_names,
      default_bt_xml_filename,
      std::bind(&BehaviorTreeNavigator::onGoalReceived, this, std::placeholders::_1),
      std::bind(&BehaviorTreeNavigator::onLoop, this),
      std::bind(&BehaviorTreeNavigator::onPreempt, this, std::placeholders::_1),
      std::bind(
        &BehaviorTreeNavigator::onCompletion, this,
        std::placeholders::_1, std::placeholders::_2));

    bool ok = true;
    if (!bt_action_server_->on_configure()) {
      ok = false;
    }

    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set("tf_buffer", feedback_utils.tf);  // NOLINT
    blackboard->set("initial_pose_received", false);  // NOLINT
    blackboard->set("number_recoveries", 0);  // NOLINT
    blackboard->set("odom_smoother", odom_smoother);  // NOLINT

    current_error_code_ = 0;
    current_error_msg_ = "";

    return configure(parent_node, odom_smoother) && ok;
  }

  /**
   * @brief Activation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool on_activate() final
  {
    bool ok = true;

    if (!bt_action_server_->on_activate()) {
      ok = false;
    }

    return activate() && ok;
  }

  /**
   * @brief Deactivation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool on_deactivate() final
  {
    bool ok = true;
    if (!bt_action_server_->on_deactivate()) {
      ok = false;
    }

    return deactivate() && ok;
  }

  /**
   * @brief Cleanup a navigator
   * @return bool If successful
   */
  bool on_cleanup() final
  {
    bool ok = true;
    if (!bt_action_server_->on_cleanup()) {
      ok = false;
    }

    bt_action_server_.reset();

    return cleanup() && ok;
  }

  virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;

  /**
   * @brief Get the action name of this navigator to expose
   * @return string Name of action to expose
   */
  virtual std::string getName() = 0;

protected:
  /**
   * @brief An intermediate goal reception function to mux navigators.
   */
  bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
  {
    if (plugin_muxer_->isNavigating()) {
      RCLCPP_ERROR(
        logger_,
        "Requested navigation from %s while another navigator is processing,"
        " rejecting request.", getName().c_str());
      return false;
    }

    bool goal_accepted = goalReceived(goal);

    if (goal_accepted) {
      plugin_muxer_->startNavigating(getName());
    }

    return goal_accepted;
  }

  /**
   * @brief An intermediate completion function to mux navigators
   */
  void onCompletion(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status)
  {
    plugin_muxer_->stopNavigating(getName());
    goalCompleted(result, final_bt_status);
  }

  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   */
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  virtual void onLoop() = 0;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that is called when a the action is completed; Can fill in
   * action result message or indicate that this action is done.
   */
  virtual void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) = 0;

  /**
   * @param Method to configure resources.
   */
  virtual bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr /*node*/,
    std::shared_ptr<nav2_util::OdomSmoother>/*odom_smoother*/)
  {
    return true;
  }

  /**
   * @brief Method to cleanup resources.
   */
  virtual bool cleanup() {return true;}

  /**
   * @brief Method to activate any threads involved in execution.
   */
  virtual bool activate() {return true;}

  /**
   * @brief Method to deactivate and any threads involved in execution.
   */
  virtual bool deactivate() {return true;}

  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("Navigator")};
  rclcpp::Clock::SharedPtr clock_;
  FeedbackUtils feedback_utils_;
  NavigatorMuxer * plugin_muxer_;

  // Error tracking
  uint16_t current_error_code_;
  std::string current_error_msg_;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__BEHAVIOR_TREE_NAVIGATOR_HPP_
