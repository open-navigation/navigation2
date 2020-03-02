// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__TRANSFORM_AVAILABLE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__TRANSFORM_AVAILABLE_CONDITION_HPP_

#include <string>
#include <chrono>
#include <cmath>
#include <atomic>
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

class TransformAvailableCondition : public BT::ConditionNode
{
public:
  TransformAvailableCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    was_found_(false)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

    getInput("child", child_frame_);
    getInput("parent", parent_frame_);

    if (child_frame_.empty() || parent_frame_.empty()) {
      RCLCPP_FATAL(
        node_->get_logger(), "Child frame (%s) or parent frame (%s) were empty.",
        child_frame_.c_str(), parent_frame_.c_str());
      exit(-1);
    }

    RCLCPP_DEBUG(node_->get_logger(), "Initialized an TransformAvailableCondition BT node");
  }

  TransformAvailableCondition() = delete;

  ~TransformAvailableCondition()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Shutting down TransformAvailableCondition BT node");
  }

  BT::NodeStatus tick() override
  {
    if (was_found_) {
      return BT::NodeStatus::SUCCESS;
    }

    std::string tf_error;
    bool found = tf_->canTransform(
      child_frame_, parent_frame_, tf2::TimePointZero, &tf_error);

    if (found) {
      was_found_ = true;
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(
      node_->get_logger(), "Transform from %s to %s was not found, tf error: %s",
      child_frame_.c_str(), parent_frame_.c_str(), tf_error.c_str());

    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("child", std::string(), "Child frame for transform"),
      BT::InputPort<std::string>("parent", std::string(), "parent frame for transform")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::atomic<bool> was_found_;

  std::string child_frame_;
  std::string parent_frame_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TransformAvailableCondition>("TransformAvailable");
}

#endif  // NAV2_BEHAVIOR_TREE__TRANSFORM_AVAILABLE_CONDITION_HPP_
