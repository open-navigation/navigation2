// Copyright (c) 2021 RoboTech Vision
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

#include "nav2_behavior_tree/plugins/action/smooth_path_action.hpp"

namespace nav2_behavior_tree
{

SmoothPathAction::SmoothPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::SmoothPath>(xml_tag_name, action_name, conf)
{
}

void SmoothPathAction::on_tick()
{
  getInput("unsmoothed_path", goal_.path);
  getInput("smoother_id", goal_.smoother_id);
  double max_smoothing_duration;
  getInput("max_smoothing_duration", max_smoothing_duration);
  goal_.max_smoothing_duration = rclcpp::Duration::from_seconds(max_smoothing_duration);
  getInput("check_for_collisions", goal_.check_for_collisions);
}

BT::NodeStatus SmoothPathAction::on_success()
{
  setOutput("smoothed_path", result_.result->path);
  setOutput("smoothing_duration", rclcpp::Duration(result_.result->smoothing_duration).seconds());
  setOutput("was_completed", result_.result->was_completed);
  // Set empty error code, action was successful
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SmoothPathAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SmoothPathAction::on_cancelled()
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
      return std::make_unique<nav2_behavior_tree::SmoothPathAction>(
        name, "smooth_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SmoothPathAction>(
    "SmoothPath", builder);
}
