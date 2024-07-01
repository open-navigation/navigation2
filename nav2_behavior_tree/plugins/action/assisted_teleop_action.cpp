// Copyright (c) 2022 Joshua Wallace
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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/assisted_teleop_action.hpp"

namespace nav2_behavior_tree
{

AssistedTeleopAction::AssistedTeleopAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::AssistedTeleop>(xml_tag_name, action_name, conf),
  initialized_(false)
{}

void AssistedTeleopAction::initialize()
{
  double time_allowance;
  getInput("time_allowance", time_allowance);
  getInput("is_recovery", is_recovery_);

  // Populate the input message
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  initialized_ = true;
}

void AssistedTeleopAction::on_tick()
{
  if (!initialized_) {
    initialize();
  }

  if (is_recovery_) {
    increment_recovery_count();
  }
}

BT::NodeStatus AssistedTeleopAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AssistedTeleopAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return is_recovery_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AssistedTeleopAction::on_cancelled()
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
      return std::make_unique<nav2_behavior_tree::AssistedTeleopAction>(
        name, "assisted_teleop", config);
    };

  factory.registerBuilder<nav2_behavior_tree::AssistedTeleopAction>("AssistedTeleop", builder);
}
