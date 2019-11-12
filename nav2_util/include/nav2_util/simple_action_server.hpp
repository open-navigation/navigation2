// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_
#define NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <future>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_util
{

template<typename ActionT, typename nodeT = rclcpp::Node>
class SimpleActionServer
{
public:
  typedef std::function<void ()> ExecuteCallback;

  explicit SimpleActionServer(
    typename nodeT::SharedPtr node,
    const std::string & action_name,
    ExecuteCallback execute_callback,
    bool autostart = true)
  : SimpleActionServer(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      action_name, execute_callback, autostart)
  {}

  explicit SimpleActionServer(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    const std::string & action_name,
    ExecuteCallback execute_callback,
    bool autostart = true)
  : node_base_interface_(node_base_interface),
    node_clock_interface_(node_clock_interface),
    node_logging_interface_(node_logging_interface),
    node_waitables_interface_(node_waitables_interface),
    action_name_(action_name), execute_callback_(execute_callback)
  {
    if (autostart) {
      server_active_ = true;
    }

    auto handle_goal =
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)
      {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!server_active_) {
          return rclcpp_action::GoalResponse::REJECT;
        }

        debug_msg("Received request for goal acceptance");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

    auto handle_cancel =
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
      {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        // TODO(orduno) could goal handle be aborted (and on a terminal state) before reaching here?
        debug_msg("Received request for goal cancellation");
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto handle_accepted =
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
      {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        debug_msg("Receiving a new goal");

        if (is_active(current_handle_)) {
          debug_msg("An older goal is active, moving the new goal to a pending slot.");

          if (is_active(pending_handle_)) {
            debug_msg("The pending slot is occupied."
              " The previous pending goal will be terminated and replaced.");
            terminate(pending_handle_);
          }
          pending_handle_ = handle;
          preempt_requested_ = true;
        } else {
          if (is_active(pending_handle_)) {
            // Shouldn't reach a state with a pending goal but no current one.
            error_msg("Forgot to handle a preemption. Terminating the pending goal.");
            terminate(pending_handle_);
            preempt_requested_ = false;
          }

          current_handle_ = handle;

          auto work = [this]() {
              while (rclcpp::ok() && !stop_execution_ && is_active(current_handle_)) {
                debug_msg("Executing the goal...");
                execute_callback_();

                debug_msg("Blocking processing of new goal handles.");
                std::lock_guard<std::recursive_mutex> lock(update_mutex_);

                if (stop_execution_) {
                  warn_msg("Stopping the thread per request.");
                  terminate(current_handle_);
                  terminate(pending_handle_);
                  preempt_requested_ = false;
                  break;
                }

                if (is_active(current_handle_)) {
                  warn_msg("Current goal was not completed successfully.");
                  terminate(current_handle_);
                }

                if (is_active(pending_handle_)) {
                  debug_msg("Executing a pending handle on the existing thread.");
                  accept_pending_goal();
                } else {
                  debug_msg("Done processing available goals.");
                  break;
                }
              }
              debug_msg("Worker thread done.");
            };

          debug_msg("Executing goal asynchronously.");
          execution_future_ = std::async(std::launch::async, work);
        }
      };

    action_server_ = rclcpp_action::create_server<ActionT>(
      node_base_interface_,
      node_clock_interface_,
      node_logging_interface_,
      node_waitables_interface_,
      action_name_,
      handle_goal,
      handle_cancel,
      handle_accepted);
  }

  void activate()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    server_active_ = true;
    stop_execution_ = false;
  }

  void deactivate()
  {
    debug_msg("Deactivating...");

    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      server_active_ = false;
      stop_execution_ = true;
    }

    if (is_running()) {
      warn_msg("Requested to deactivate server but goal is still executing."
        " Should check if action server is running before deactivating.");
    }

    using namespace std::chrono_literals;  //NOLINT
    while (execution_future_.wait_for(100ms) != std::future_status::ready) {
      info_msg("Waiting for async process to finish.");
    }
    debug_msg("Deactivation completed.");
  }

  bool is_running()
  {
    using namespace std::chrono_literals;  //NOLINT
    return execution_future_.wait_for(0ms) == std::future_status::ready ? true : false;
  }

  bool is_server_active()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return server_active_;
  }

  bool is_preempt_requested() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return preempt_requested_;
  }

  const std::shared_ptr<const typename ActionT::Goal> accept_pending_goal()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      error_msg("Attempting to get pending goal when not available");
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    if (is_active(current_handle_) && current_handle_ != pending_handle_) {
      debug_msg("Cancelling the previous goal");
      current_handle_->abort(empty_result());
    }

    current_handle_ = pending_handle_;
    pending_handle_.reset();
    preempt_requested_ = false;

    debug_msg("Preempted goal");

    return current_handle_->get_goal();
  }

  const std::shared_ptr<const typename ActionT::Goal> get_current_goal() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!is_active(current_handle_)) {
      error_msg("A goal is not available or has reached a final state");
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    return current_handle_->get_goal();
  }

  bool is_cancel_requested() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    // A cancel request is assumed if either handle is canceled by the client.

    if (current_handle_ == nullptr) {
      error_msg("Checking for cancel but current goal is not available");
      return false;
    }

    if (pending_handle_ != nullptr) {
      return pending_handle_->is_canceling();
    }

    return current_handle_->is_canceling();
  }

  void terminate(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle,
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (is_active(handle)) {
      if (handle->is_canceling()) {
        warn_msg("Client requested to cancel the current goal. Cancelling.");
        handle->canceled(result);
      } else {
        warn_msg("Aborting handle.");
        handle->abort(result);
      }
      handle.reset();
    }
  }

  void succeeded_current(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (is_active(current_handle_)) {
      debug_msg("Setting succeed on current goal.");
      current_handle_->succeed(result);
      current_handle_.reset();
    }
  }

  void publish_feedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
  {
    if (!is_active(current_handle_)) {
      error_msg("Trying to publish feedback when the current goal handle is not active");
    }

    current_handle_->publish_feedback(feedback);
  }

protected:
  // The SimpleActionServer isn't itself a node, so it needs interfaces to one
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;
  std::string action_name_;

  ExecuteCallback execute_callback_;
  std::future<void> execution_future_;
  bool stop_execution_;

  mutable std::recursive_mutex update_mutex_;
  bool server_active_{false};
  bool preempt_requested_{false};
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> pending_handle_;

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;

  constexpr auto empty_result() const
  {
    return std::make_shared<typename ActionT::Result>();
  }

  constexpr bool is_active(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle) const
  {
    return handle != nullptr && handle->is_active();
  }

  void info_msg(const std::string & msg) const
  {
    RCLCPP_INFO(node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  void debug_msg(const std::string & msg) const
  {
    RCLCPP_DEBUG(node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  void error_msg(const std::string & msg) const
  {
    RCLCPP_ERROR(node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  void warn_msg(const std::string & msg) const
  {
    RCLCPP_WARN(node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }
};

}  // namespace nav2_util

#endif   // NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_
