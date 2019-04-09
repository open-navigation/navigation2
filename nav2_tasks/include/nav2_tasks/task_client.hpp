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

#ifndef NAV2_TASKS__TASK_CLIENT_HPP_
#define NAV2_TASKS__TASK_CLIENT_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_lifecycle/lifecycle_node.hpp"
#include "nav2_tasks/task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

namespace nav2_tasks
{

constexpr std::chrono::milliseconds defaultServerTimeout = std::chrono::milliseconds(5000);

template<typename CommandMsg, typename ResultMsg>
const char * getTaskName();

template<typename CommandMsg, typename ResultMsg>
class TaskClient : public nav2_lifecycle::LifecycleHelperInterface
{
public:
  explicit TaskClient(nav2_lifecycle::LifecycleNode::SharedPtr node, bool autoinit = false)
  : node_(node), autoinit_(autoinit)
  {
    resultReceived_ = false;
    statusReceived_ = false;

    statusMsg_ = std::make_shared<StatusMsg>();

    // There are some cases where the TaskClient is used from a context that doesn't have
    // a lifecycle-style inteface (such as BehaviorTrees). For those situations, the
    // TaskClient class can be automatically configured and activated

    if (autoinit_) {
      rclcpp_lifecycle::State state0(
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "unconfigured");
      on_configure(state0);

      rclcpp_lifecycle::State state1(
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
      on_activate(state1);
    }
  }

  TaskClient() = delete;

  ~TaskClient()
  {
    if (autoinit_) {
      rclcpp_lifecycle::State state0(
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
      on_deactivate(state0);

      rclcpp_lifecycle::State state1(
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
      on_cleanup(state1);
    }
  }

  nav2_lifecycle::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    std::string taskName = getTaskName<CommandMsg, ResultMsg>();

    // Create the publishers
    commandPub_ = node_->create_publisher<CommandMsg>(taskName + "_command");
    updatePub_ = node_->create_publisher<CommandMsg>(taskName + "_update");
    cancelPub_ = node_->create_publisher<CancelMsg>(taskName + "_cancel");

    // Create the subscribers
    resultSub_ = node_->create_subscription<ResultMsg>(taskName + "_result",
        std::bind(&TaskClient::onResultReceived, this, std::placeholders::_1));
    statusSub_ = node_->create_subscription<StatusMsg>(taskName + "_status",
        std::bind(&TaskClient::onStatusReceived, this, std::placeholders::_1));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  nav2_lifecycle::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    commandPub_->on_activate();
    updatePub_->on_activate();
    cancelPub_->on_activate();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  nav2_lifecycle::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    commandPub_->on_deactivate();
    updatePub_->on_deactivate();
    cancelPub_->on_deactivate();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  nav2_lifecycle::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    commandPub_.reset();
    updatePub_.reset();
    cancelPub_.reset();

    resultSub_.reset();
    statusSub_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // The client can tell the TaskServer to execute its operation
  void sendCommand(const typename CommandMsg::SharedPtr msg)
  {
    resultReceived_ = false;
    statusReceived_ = false;

    commandPub_->publish(msg);
  }

  void sendUpdate(const typename CommandMsg::SharedPtr msg)
  {
    resultReceived_ = false;
    statusReceived_ = false;

    updatePub_->publish(msg);
  }

  // An in-flight operation can be canceled
  void cancel()
  {
    CancelMsg msg;
    cancelPub_->publish(msg);
  }

  bool waitForServer(std::chrono::milliseconds timeout = std::chrono::milliseconds::max())
  {
    std::string taskName = getTaskName<CommandMsg, ResultMsg>();
    taskName += "_command";

    auto t0 = std::chrono::high_resolution_clock::now();

    while (node_->count_subscribers(taskName) < 1) {
      auto t1 = std::chrono::high_resolution_clock::now();
      auto elapsedTime = t1 - t0;

      if (elapsedTime > timeout) {
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return true;
  }

  // The client can wait for a result with a timeout
  TaskStatus waitForResult(
    typename ResultMsg::SharedPtr & result,
    std::chrono::milliseconds duration)
  {
    // Wait for a status message to come in
    std::unique_lock<std::mutex> lock(statusMutex_);
    if (!cvStatus_.wait_for(lock, std::chrono::milliseconds(duration),
      [&] {return statusReceived_ == true;}))
    {
      return RUNNING;
    }

    // We've got a status message, indicating that the server task has finished (succeeded,
    // failed, or canceled)
    switch (statusMsg_->result) {
      // If the task has failed or has been canceled, no result message is forthcoming and we
      // can propagate the status code, using the TaskStatus type rather than the message-level
      // implementation type
      case nav2_msgs::msg::TaskStatus::FAILED:
      case nav2_msgs::msg::TaskStatus::CANCELED:
        return static_cast<TaskStatus>(statusMsg_->result);

      case nav2_msgs::msg::TaskStatus::SUCCEEDED:
        {
          // The result message may be here already or it may come *after* the status
          // message. If it's here, the wait will be satisfied immediately. Otherwise
          // we'll wait for a bit longer to see if it comes in.
          std::unique_lock<std::mutex> lock(resultMutex_);
          if (cvResult_.wait_for(lock, std::chrono::milliseconds(100),
            [&] {return resultReceived_ == true;}))
          {
            // Copy only the data, not the pointer since the pointer may have been used in
            // a BT blackboard
            *result = *resultMsg_;

            resultReceived_ = false;
            return SUCCEEDED;
          }

          // Give up since we never received the result message
          return FAILED;
        }

      default:
        throw std::logic_error("Invalid status value from TaskServer");
    }

    // Not reachable; added to avoid a warning
    return FAILED;
  }

protected:
  // The result of this task
  typename ResultMsg::SharedPtr resultMsg_;

  // These messages are internal to the TaskClient implementation
  typedef std_msgs::msg::Empty CancelMsg;
  typedef nav2_msgs::msg::TaskStatus StatusMsg;
  StatusMsg::SharedPtr statusMsg_;

  // Variables to handle the communication of the status message to the waitForResult thread
  std::mutex statusMutex_;
  std::atomic<bool> statusReceived_;
  std::condition_variable cvStatus_;

  // Variables to handle the communication of the result message to the waitForResult thread
  std::mutex resultMutex_;
  std::atomic<bool> resultReceived_;
  std::condition_variable cvResult_;

  // Called when the TaskServer has sent its result
  void onResultReceived(const typename ResultMsg::SharedPtr resultMsg)
  {
    {
      std::lock_guard<std::mutex> lock(resultMutex_);
      resultMsg_ = resultMsg;
      resultReceived_ = true;
    }

    cvResult_.notify_one();
  }

  // Called when the TaskServer sends its status code (success or failure)
  void onStatusReceived(const StatusMsg::SharedPtr statusMsg)
  {
    {
      std::lock_guard<std::mutex> lock(statusMutex_);
      statusMsg_ = statusMsg;
      statusReceived_ = true;
    }

    cvStatus_.notify_one();
  }

  // The TaskClient isn't itself a node, so needs to know which one to use
  nav2_lifecycle::LifecycleNode::SharedPtr node_;

  // The client's publishers: the command and cancel messages
  typename rclcpp_lifecycle::LifecyclePublisher<CommandMsg>::SharedPtr commandPub_;
  typename rclcpp_lifecycle::LifecyclePublisher<CommandMsg>::SharedPtr updatePub_;
  rclcpp_lifecycle::LifecyclePublisher<CancelMsg>::SharedPtr cancelPub_;

  // The client's subscriptions: result, feedback, and status
  typename rclcpp::Subscription<ResultMsg>::SharedPtr resultSub_;
  rclcpp::Subscription<StatusMsg>::SharedPtr statusSub_;

  // Whether to automatically walk the pubs through the lifecycle states
  bool autoinit_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__TASK_CLIENT_HPP_
