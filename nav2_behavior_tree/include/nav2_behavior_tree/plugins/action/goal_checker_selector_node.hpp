// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_CHECKER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_CHECKER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief The GoalCheckerSelector behavior is used to switch the goal checker
 * of the controller server. It subscribes to a topic "goal_checker_selector"
 * to get the decision about what goal_checker must be used. It is usually used before of
 * the FollowPath. The selected_goal_checker output port is passed to goal_checker_id
 * input port of the FollowPath
 */
class GoalCheckerSelector : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GoalCheckerSelector
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   */
  GoalCheckerSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "default_goal_checker",
        "the default goal_checker to use if there is not any external topic message received."),

      BT::InputPort<std::string>(
        "topic_name",
        "goal_checker_selector",
        "the input topic name to select the goal_checker"),

      BT::OutputPort<std::string>(
        "selected_goal_checker",
        "Selected goal_checker by subscription")
    };
  }

private:
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief callback function for the goal_checker_selector topic
   *
   * @param msg the message with the id of the goal_checker_selector
   */
  void callbackGoalCheckerSelect(const std_msgs::msg::String::SharedPtr msg);

  bool first_time;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_checker_selector_sub_;

  std::string last_selected_goal_checker_;

  rclcpp::Node::SharedPtr node_;

  std::string topic_name_;
  bool initialized_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_CHECKER_SELECTOR_NODE_HPP_
