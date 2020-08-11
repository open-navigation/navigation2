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

#ifndef NAV2_RECOVERIES__RECOVERY_HPP_
#define NAV2_RECOVERIES__RECOVERY_HPP_

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_core/recovery.hpp"

namespace nav2_recoveries
{

enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

using namespace std::chrono_literals;  //NOLINT

template<typename ActionT>
class Recovery : public nav2_core::Recovery
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT, rclcpp_lifecycle::LifecycleNode>;

  Recovery()
  : action_server_(nullptr),
    cycle_frequency_(10.0),
    enabled_(false)
  {
  }

  virtual ~Recovery()
  {
  }

  // Derived classes can override this method to catch the command and perform some checks
  // before getting into the main loop. The method will only be called
  // once and should return SUCCEEDED otherwise behavior will return FAILED.
  virtual Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) = 0;


  // This is the method derived classes should mainly implement
  // and will be called cyclically while it returns RUNNING.
  // Implement the behavior such that it runs some unit of work on each call
  // and provides a status. The recovery will finish once SUCCEEDED is returned
  // It's up to the derived class to define the final commanded velocity.
  virtual Status onCycleUpdate() = 0;

  // an opportunity for derived classes to do something on configuration
  // if they chose
  virtual void onConfigure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & /*node*/)
  {
  }

  // an opportunity for derived classes to do something on cleanup
  // if they chose
  virtual void onCleanup()
  {
  }

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) override
  {
    RCLCPP_INFO(node->get_logger(), "Configuring %s", name.c_str());

    node_base_interface_ = node->get_node_base_interface();
    node_logging_interface_ = node->get_node_logging_interface();
    node_timers_interface_ = node->get_node_timers_interface();

    recovery_name_ = name;
    tf_ = tf;

    node->get_parameter("cycle_frequency", cycle_frequency_);
    node->get_parameter("global_frame", global_frame_);
    node->get_parameter("robot_base_frame", robot_base_frame_);
    node->get_parameter("transform_tolerance", transform_tolerance_);

    action_server_ = std::make_shared<ActionServer>(
      node, recovery_name_,
      std::bind(&Recovery::execute, this));

    collision_checker_ = collision_checker;

    vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    onConfigure(node);
  }

  void cleanup() override
  {
    action_server_.reset();
    vel_pub_.reset();
    onCleanup();
  }

  void activate() override
  {
    RCLCPP_INFO(node_logging_interface_->get_logger(), "Activating %s", recovery_name_.c_str());

    vel_pub_->on_activate();
    action_server_->activate();
    enabled_ = true;
  }

  void deactivate() override
  {
    vel_pub_->on_deactivate();
    action_server_->deactivate();
    enabled_ = false;
  }

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_interface_;

  std::string recovery_name_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  std::shared_ptr<ActionServer> action_server_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  double cycle_frequency_;
  double enabled_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  void execute()
  {
    RCLCPP_INFO(node_logging_interface_->get_logger(), "Attempting %s", recovery_name_.c_str());

    if (!enabled_) {
      RCLCPP_WARN(
        node_logging_interface_->get_logger(),
        "Called while inactive, ignoring request.");
      return;
    }

    if (onRun(action_server_->get_current_goal()) != Status::SUCCEEDED) {
      RCLCPP_INFO(
        node_logging_interface_->get_logger(),
        "Initial checks failed for %s", recovery_name_.c_str());
      action_server_->terminate_current();
      return;
    }

    // Log a message every second
    auto timer = rclcpp::create_wall_timer(
      1s,
      [&]() {
        RCLCPP_INFO(
          node_logging_interface_->get_logger(),
          "%s running...", recovery_name_.c_str());
      },
      nullptr,
      node_base_interface_.get(),
      node_timers_interface_.get());

    auto start_time = steady_clock_.now();

    // Initialize the ActionT result
    auto result = std::make_shared<typename ActionT::Result>();

    rclcpp::Rate loop_rate(cycle_frequency_);

    while (rclcpp::ok()) {
      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(node_logging_interface_->get_logger(), "Canceling %s", recovery_name_.c_str());
        stopRobot();
        result->total_elapsed_time = steady_clock_.now() - start_time;
        action_server_->terminate_all(result);
        return;
      }

      // TODO(orduno) #868 Enable preempting a Recovery on-the-fly without stopping
      if (action_server_->is_preempt_requested()) {
        RCLCPP_ERROR(
          node_logging_interface_->get_logger(), "Received a preemption request for %s,"
          " however feature is currently not implemented. Aborting and stopping.",
          recovery_name_.c_str());
        stopRobot();
        result->total_elapsed_time = steady_clock_.now() - start_time;
        action_server_->terminate_current(result);
        return;
      }

      switch (onCycleUpdate()) {
        case Status::SUCCEEDED:
          RCLCPP_INFO(
            node_logging_interface_->get_logger(),
            "%s completed successfully", recovery_name_.c_str());
          result->total_elapsed_time = steady_clock_.now() - start_time;
          action_server_->succeeded_current(result);
          return;

        case Status::FAILED:
          RCLCPP_WARN(node_logging_interface_->get_logger(), "%s failed", recovery_name_.c_str());
          result->total_elapsed_time = steady_clock_.now() - start_time;
          action_server_->terminate_current(result);
          return;

        case Status::RUNNING:

        default:
          loop_rate.sleep();
          break;
      }
    }
  }

  void stopRobot()
  {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = 0.0;
    cmd_vel->linear.y = 0.0;
    cmd_vel->angular.z = 0.0;

    vel_pub_->publish(std::move(cmd_vel));
  }
};

}  // namespace nav2_recoveries

#endif  // NAV2_RECOVERIES__RECOVERY_HPP_
