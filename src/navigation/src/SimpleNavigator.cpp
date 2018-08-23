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


// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

#include <string>
#include <memory>
#include <exception>
#include <chrono>
#include "navigation/SimpleNavigator.hpp"

SimpleNavigator::SimpleNavigator(const std::string & name)
: NavigateToPoseTaskServer(name)
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::SimpleNavigator");
  planner_ = std::make_unique<ComputePathToPoseTaskClient>("DijkstraPlanner", this);
  controller_ = std::make_unique<FollowPathTaskClient>("DwaController", this);
}

SimpleNavigator::~SimpleNavigator()
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::~SimpleNavigator");
}

TaskStatus
SimpleNavigator::executeAsync(const NavigateToPoseCommand::SharedPtr /*command*/)
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync");

  // Compose the PathEndPoints message for Navigation
  auto endpoints = std::make_shared<ComputePathToPoseCommand>();
  // TODO(mdjeroni): get the starting pose from Localization (fake it out for now)
  endpoints->start.position.x = 1.0;
  endpoints->start.position.y = 1.0;
  endpoints->goal.position.x = 9.0;
  endpoints->goal.position.y = 9.0;
  endpoints->tolerance = 2.0;

  RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: getting the path from the planner");
  auto path = std::make_shared<ComputePathToPoseResult>();
  planner_->executeAsync(endpoints);

  // TODO(orduno): implement continous replanning

  // Loop until the subtasks are completed
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: task has been canceled");
      planner_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // Check if the planning task has completed
    TaskStatus status = planner_->waitForResult(path, 100);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: planning task completed");
        goto here;

      case TaskStatus::FAILED:
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: planning task still running");
        break;

      default:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: invalid status value");
        throw std::logic_error("SimpleNavigator::executeAsync: invalid status value");
    }
  }

here:

  RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: got path of size %u",
    path->poses.size());
  int index;
  for (auto pose : path->poses) {
    RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: point %u x: %0.2f, y: %0.2f",
      index, pose.position.x, pose.position.y);
    index++;
  }

  RCLCPP_INFO(get_logger(),
    "SimpleNavigator::executeAsync: sending the path to the controller to execute");

  controller_->executeAsync(path);

  // Loop until the control task completes
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: task has been canceled");
      controller_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // Check if the control task has completed
    auto controlResult = std::make_shared<FollowPathResult>();
    TaskStatus status = controller_->waitForResult(controlResult, 10);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        {
          RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: control task completed");
          NavigateToPoseResult navigationResult;
          setResult(navigationResult);

          return TaskStatus::SUCCEEDED;
        }

      case TaskStatus::FAILED:
        return TaskStatus::FAILED;

      case TaskStatus::RUNNING:
        // RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: control task still running");
        break;

      default:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: invalid status value");
        throw std::logic_error("SimpleNavigator::executeAsync: invalid status value");
    }
  }
}
