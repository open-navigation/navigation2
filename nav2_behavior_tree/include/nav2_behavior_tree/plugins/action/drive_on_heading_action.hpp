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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DRIVE_ON_HEADING_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DRIVE_ON_HEADING_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::DriveOnHeading
 */
class DriveOnHeadingAction : public BtActionNode<nav2_msgs::action::DriveOnHeading>
{
  using Action = nav2_msgs::action::DriveOnHeading;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::DriveOnHeadingAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  DriveOnHeadingAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
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
    return providedBasicPorts(
      {
        BT::InputPort<double>("dist_to_travel", 0.15, "Distance to travel"),
        BT::InputPort<double>("speed", 0.025, "Speed at which to travel"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for driving on heading"),
        BT::OutputPort<Action::Result::_error_code_type>(
          "error_code_id", "The drive on heading behavior server error code"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_msg", "The drive on heading behavior server error msg"),
      });
  }

  /**
 * @brief Function to perform some user-defined operation on tick
 */
  void on_tick() override;

  /**
 * @brief Function to perform some user-defined operation upon successful completion of the action
 */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancellation of the action
   */
  BT::NodeStatus on_cancelled() override;

private:
  bool initalized_;
};

}  // namespace nav2_behavior_tree
#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DRIVE_ON_HEADING_ACTION_HPP_
