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

#include "nav2_behavior_tree/plugins/action/spin_action.hpp"

namespace nav2_behavior_tree
{

SpinAction::SpinAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf),
  initialized_(false)
{
}

void SpinAction::initialize()
{
  double dist;
  getInput("spin_dist", dist);
  double time_allowance;
  getInput("time_allowance", time_allowance);
  goal_.target_yaw = dist;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  getInput("is_recovery", is_recovery_);

  initialized_ = true;
}

void SpinAction::on_tick()
{
  if (!initialized_) {
    initialize();
  }

  if (is_recovery_) {
    increment_recovery_count();
  }
}

BT::NodeStatus SpinAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SpinAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SpinAction::on_cancelled()
{
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
      return std::make_unique<nav2_behavior_tree::SpinAction>(name, "spin", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SpinAction>("Spin", builder);
}
