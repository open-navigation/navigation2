// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__UTILS__TEST_ACTION_SERVER_HPP_
#define NAV2_BEHAVIOR_TREE__UTILS__TEST_ACTION_SERVER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

template<class ActionT>
class TestActionServer : public rclcpp::Node
{
public:
  explicit TestActionServer(
    std::string action_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("test_action_server", options)
  {
    using namespace std::placeholders;  // NOLINT

    this->action_server_ = rclcpp_action::create_server<ActionT>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      action_name,
      std::bind(&TestActionServer::handle_goal, this, _1, _2),
      std::bind(&TestActionServer::handle_cancel, this, _1),
      std::bind(&TestActionServer::handle_accepted, this, _1));
  }

  std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
  {
    return current_goal_;
  }

  void setReturnSuccess(bool return_success)
  {
    return_success_ = return_success;
  }

  bool getReturnSuccess(void)
  {
    return return_success_;
  }

  bool isGoalCancelled()
  {
    return goal_cancelled_;
  }

protected:
  virtual rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const typename ActionT::Goal> goal)
  {
    current_goal_ = goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  virtual rclcpp_action::CancelResponse handle_cancel(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
  {
    goal_cancelled_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  virtual void execute(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle) = 0;

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    using namespace std::placeholders;  // NOLINT
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TestActionServer::execute, this, _1), goal_handle}.detach();
  }

private:
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  std::shared_ptr<const typename ActionT::Goal> current_goal_;
  bool return_success_ = true;
  bool goal_cancelled_ = false;
};

#endif  // NAV2_BEHAVIOR_TREE__UTILS__TEST_ACTION_SERVER_HPP_
