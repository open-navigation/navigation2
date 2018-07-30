// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "navigation/SimpleNavigator.hpp"
#include <chrono>

SimpleNavigator::SimpleNavigator(const std::string & name, Robot * /*robot*/)
: NavigateToPoseTaskServer(name)
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::SimpleNavigator");
  planner_ = std::make_unique<PlanningTaskClient>("AStarPlanner", this);
  controller_ = std::make_unique<ControlTaskClient>("DwaController", this);
}

SimpleNavigator::~SimpleNavigator()
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::~SimpleNavigator");
}

TaskStatus
SimpleNavigator::execute(const std_msgs::msg::String::SharedPtr /*command*/)
{
  RCLCPP_INFO(get_logger(), "SimpleNavigator::execute");

  RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: getting the path from the planner");
  planner_->execute();

  // Simulate looping until the planner reaches a terminal state
  for (;;) {
    // Check to see if this task has been canceled. If so, cancel any child tasks
	// and bail out
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: task has been canceled");
      planner_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }

    // Otherwise, check if the child task has completed (succeeded or failed)
    std_msgs::msg::String planningResult;
	TaskStatus status = planner_->waitForResult(planningResult, 100);

	switch (status)
	{
	  case TaskStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: planning task completed");
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: msg: %s", planningResult.data.c_str());

		goto here;

	  case TaskStatus::FAILED:
        return TaskStatus::FAILED;

	  case TaskStatus::RUNNING:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: planning task still running");
		break;

	  default:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: invalid status value");
        throw("SimpleNavigator::execute: invalid status value");
    }
  }

here:
  RCLCPP_INFO(get_logger(),
      "SimpleNavigator::execute: sending the path to the controller to execute");

  controller_->execute();

  // Simulate looping until the controller reaches a terminal state
  for (;;) {
    // Check to see if this task has been canceled. If so, cancel any child tasks
	// and bail out
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: task has been canceled");
      controller_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
	}

    // Otherwise, check if the child task has completed (succeeded or failed)
    std_msgs::msg::String controlResult;
	TaskStatus status = controller_->waitForResult(controlResult, 10);

	switch (status)
	{
	  case TaskStatus::SUCCEEDED:
	  {
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: control task completed");
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: msg: %s", controlResult.data.c_str());
        std_msgs::msg::String navigationResult;
        navigationResult.data = "Here is the result from the SimpleNavigator!";
        setResult(navigationResult);

        return TaskStatus::SUCCEEDED;
      }

	  case TaskStatus::FAILED:
        return TaskStatus::FAILED;

	  case TaskStatus::RUNNING:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: control task still running");
		break;

	  default:
        RCLCPP_INFO(get_logger(), "SimpleNavigator::execute: invalid status value");
        throw("SimpleNavigator::execute: invalid status value");
    }
  }
}
