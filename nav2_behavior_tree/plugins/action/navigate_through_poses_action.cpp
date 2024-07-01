// Copyright (c) 2021 Samsung Research America
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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/navigate_through_poses_action.hpp"

namespace nav2_behavior_tree
{

NavigateThroughPosesAction::NavigateThroughPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateThroughPoses>(xml_tag_name, action_name, conf)
{
}

void NavigateThroughPosesAction::on_tick()
{
  if (!getInput("goals", goal_.poses)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "NavigateThroughPosesAction: goal not provided");
    return;
  }
  getInput("behavior_tree", goal_.behavior_tree);
}

BT::NodeStatus NavigateThroughPosesAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus NavigateThroughPosesAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavigateThroughPosesAction::on_cancelled()
{
  // Set empty error code, action was cancelled
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::NavigateThroughPosesAction>(
        name, "navigate_through_poses", config);
    };

  factory.registerBuilder<nav2_behavior_tree::NavigateThroughPosesAction>(
    "NavigateThroughPoses", builder);
}
