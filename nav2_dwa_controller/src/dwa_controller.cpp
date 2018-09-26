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

#include <string>
#include <chrono>
#include "nav2_dwa_controller/dwa_controller.hpp"

using namespace std::chrono_literals;
using nav2_tasks::TaskStatus;

namespace nav2_dwa_controller
{

DwaController::DwaController()
: nav2_tasks::FollowPathTaskServer("FollowPathNode")
{
  RCLCPP_INFO(get_logger(), "DwaController::DwaController");
}

DwaController::~DwaController()
{
  RCLCPP_INFO(get_logger(), "DwaController::~DwaController");
}

TaskStatus
DwaController::execute(const nav2_tasks::FollowPathCommand::SharedPtr /*command*/)
{
  RCLCPP_INFO(get_logger(), "DwaController::execute");

  // Spin here for a bit to fake out some processing time
  for (int i = 0; i < 10; i++) {
    // Do a bit of the task
    RCLCPP_INFO(get_logger(), "DwaController::execute: doing work: %d", i);
    std::this_thread::sleep_for(250ms);

    // Before we loop again to do more work, check if we've been canceled
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "DwaController::execute: task has been canceled");
      setCanceled();
      return TaskStatus::CANCELED;
    }
  }

  // We've successfully completed the task, so return the result
  RCLCPP_INFO(get_logger(), "DwaController::execute: task completed");

  nav2_tasks::FollowPathResult result;
  setResult(result);

  return TaskStatus::SUCCEEDED;
}

}  // namespace nav2_dwa_controller
