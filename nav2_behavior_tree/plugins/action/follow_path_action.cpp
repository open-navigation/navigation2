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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"

namespace nav2_behavior_tree
{

FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void FollowPathAction::on_tick()
{
  getInput("path", goal_.path);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);
  getInput("progress_checker_id", goal_.progress_checker_id);
}

BT::NodeStatus FollowPathAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FollowPathAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowPathAction::on_cancelled()
{
  // Set empty error code, action was cancelled
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void FollowPathAction::on_wait_for_result(
  std::shared_ptr<const Action::Feedback>/*feedback*/)
{
  // Grab the new path
  nav_msgs::msg::Path new_path;
  getInput("path", new_path);

  // Check if it is not same with the current one
  if (goal_.path != new_path) {
    // the action server on the next loop iteration
    goal_.path = new_path;
    goal_updated_ = true;
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);

  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }

  std::string new_progress_checker_id;
  getInput("progress_checker_id", new_progress_checker_id);

  if (goal_.progress_checker_id != new_progress_checker_id) {
    goal_.progress_checker_id = new_progress_checker_id;
    goal_updated_ = true;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FollowPathAction>(
    "FollowPath", builder);
}
