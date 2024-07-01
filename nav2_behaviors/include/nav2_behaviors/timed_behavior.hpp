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

#ifndef NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_
#define NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_


#include <cstdint>
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
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/twist_publisher.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_core/behavior.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop


namespace nav2_behaviors
{

enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

struct ResultStatus
{
  Status status;
  uint16_t error_code{0};
};

using namespace std::chrono_literals;  //NOLINT

/**
 * @class nav2_behaviors::Behavior
 * @brief An action server Behavior base class implementing the action server and basic factory.
 */
template<typename ActionT>
class TimedBehavior : public nav2_core::Behavior
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  /**
   * @brief A TimedBehavior constructor
   */
  TimedBehavior()
  : action_server_(nullptr),
    cycle_frequency_(10.0),
    enabled_(false),
    transform_tolerance_(0.0)
  {
  }

  virtual ~TimedBehavior() = default;

  // Derived classes can override this method to catch the command and perform some checks
  // before getting into the main loop. The method will only be called
  // once and should return SUCCEEDED otherwise behavior will return FAILED.
  virtual ResultStatus onRun(const std::shared_ptr<const typename ActionT::Goal> command) = 0;


  // This is the method derived classes should mainly implement
  // and will be called cyclically while it returns RUNNING.
  // Implement the behavior such that it runs some unit of work on each call
  // and provides a status. The Behavior will finish once SUCCEEDED is returned
  // It's up to the derived class to define the final commanded velocity.
  virtual ResultStatus onCycleUpdate() = 0;

  // an opportunity for derived classes to do something on configuration
  // if they chose
  virtual void onConfigure()
  {
  }

  // an opportunity for derived classes to do something on cleanup
  // if they chose
  virtual void onCleanup()
  {
  }

  // an opportunity for a derived class to do something on action completion
  virtual void onActionCompletion(std::shared_ptr<typename ActionT::Result>/*result*/)
  {
  }

  // configure the server on lifecycle setup
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> local_collision_checker,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> global_collision_checker)
  override
  {
    node_ = parent;
    auto node = node_.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    behavior_name_ = name;
    tf_ = tf;

    node->get_parameter("cycle_frequency", cycle_frequency_);
    node->get_parameter("local_frame", local_frame_);
    node->get_parameter("global_frame", global_frame_);
    node->get_parameter("robot_base_frame", robot_base_frame_);
    node->get_parameter("transform_tolerance", transform_tolerance_);

    if (!node->has_parameter("action_server_result_timeout")) {
      node->declare_parameter("action_server_result_timeout", 10.0);
    }

    double action_server_result_timeout;
    node->get_parameter("action_server_result_timeout", action_server_result_timeout);
    rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
    server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

    action_server_ = std::make_shared<ActionServer>(
      node, behavior_name_,
      std::bind(&TimedBehavior::execute, this), nullptr, std::chrono::milliseconds(
        500), false, server_options);

    local_collision_checker_ = local_collision_checker;
    global_collision_checker_ = global_collision_checker;

    vel_pub_ = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel", 1);

    onConfigure();
  }

  // Cleanup server on lifecycle transition
  void cleanup() override
  {
    action_server_.reset();
    vel_pub_.reset();
    onCleanup();
  }

  // Activate server on lifecycle transition
  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating %s", behavior_name_.c_str());

    vel_pub_->on_activate();
    action_server_->activate();
    enabled_ = true;
  }

  // Deactivate server on lifecycle transition
  void deactivate() override
  {
    vel_pub_->on_deactivate();
    action_server_->deactivate();
    enabled_ = false;
  }

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string behavior_name_;
  std::unique_ptr<nav2_util::TwistPublisher> vel_pub_;
  std::shared_ptr<ActionServer> action_server_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> local_collision_checker_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> global_collision_checker_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  double cycle_frequency_;
  double enabled_;
  std::string local_frame_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  rclcpp::Duration elasped_time_{0, 0};

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_behaviors")};

  // Main execution callbacks for the action server implementation calling the Behavior's
  // onRun and cycle functions to execute a specific behavior
  void execute()
  {
    RCLCPP_INFO(logger_, "Running %s", behavior_name_.c_str());

    if (!enabled_) {
      RCLCPP_WARN(
        logger_,
        "Called while inactive, ignoring request.");
      return;
    }

    // Initialize the ActionT result
    auto result = std::make_shared<typename ActionT::Result>();

    ResultStatus on_run_result = onRun(action_server_->get_current_goal());
    if (on_run_result.status != Status::SUCCEEDED) {
      result->error_msg = "Initial checks failed for " + behavior_name_;
      RCLCPP_INFO(logger_, result->error_msg.c_str());
      result->error_code = on_run_result.error_code;
      action_server_->terminate_current(result);
      return;
    }

    auto start_time = clock_->now();
    rclcpp::WallRate loop_rate(cycle_frequency_);

    while (rclcpp::ok()) {
      elasped_time_ = clock_->now() - start_time;
      if (action_server_->is_cancel_requested()) {
        result->error_msg = "Canceling " + behavior_name_;
        RCLCPP_INFO(logger_, result->error_msg.c_str());
        stopRobot();
        result->total_elapsed_time = elasped_time_;
        onActionCompletion(result);
        action_server_->terminate_all(result);
        return;
      }

      // TODO(orduno) #868 Enable preempting a Behavior on-the-fly without stopping
      if (action_server_->is_preempt_requested()) {
        RCLCPP_ERROR(
          logger_, "Received a preemption request for %s,"
          " however feature is currently not implemented. Aborting and stopping.",
          behavior_name_.c_str());
        stopRobot();
        result->total_elapsed_time = clock_->now() - start_time;
        onActionCompletion(result);
        action_server_->terminate_current(result);
        return;
      }

      ResultStatus on_cycle_update_result = onCycleUpdate();
      switch (on_cycle_update_result.status) {
        case Status::SUCCEEDED:
          RCLCPP_INFO(
            logger_,
            "%s completed successfully", behavior_name_.c_str());
          result->total_elapsed_time = clock_->now() - start_time;
          onActionCompletion(result);
          action_server_->succeeded_current(result);
          return;

        case Status::FAILED:
          result->error_msg = behavior_name_ + " failed";
          RCLCPP_WARN(logger_, result->error_msg.c_str());
          result->total_elapsed_time = clock_->now() - start_time;
          result->error_code = on_cycle_update_result.error_code;
          onActionCompletion(result);
          action_server_->terminate_current(result);
          return;

        case Status::RUNNING:

        default:
          loop_rate.sleep();
          break;
      }
    }
  }

  // Stop the robot with a commanded velocity
  void stopRobot()
  {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
    cmd_vel->header.frame_id = robot_base_frame_;
    cmd_vel->header.stamp = clock_->now();
    cmd_vel->twist.linear.x = 0.0;
    cmd_vel->twist.linear.y = 0.0;
    cmd_vel->twist.angular.z = 0.0;

    vel_pub_->publish(std::move(cmd_vel));
  }
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_
