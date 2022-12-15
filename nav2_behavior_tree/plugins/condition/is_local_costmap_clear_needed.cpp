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

#include "nav2_behavior_tree/plugins/condition/is_local_costmap_clear_needed.hpp"
#include <memory>

namespace nav2_behavior_tree
{

IsLocalCostmapClearNeeded::IsLocalCostmapClearNeeded(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: GenericErrorCodeCondition(condition_name, conf)
{
  error_codes_to_check_ = {
    ActionGoal::UNKNOWN,
    ActionGoal::PATIENCE_EXCEEDED,
    ActionGoal::FAILED_TO_MAKE_PROGRESS,
    ActionGoal::NO_VALID_CONTROL
  };
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsLocalCostmapClearNeeded>(
    "IsLocalCostmapClearNeeded");
}
