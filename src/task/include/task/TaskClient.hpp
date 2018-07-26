// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__TASKCLIENT_HPP_
#define TASK__TASKCLIENT_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TaskClient
{
public:
  TaskClient(const std::string & name, rclcpp::Node * node);
  virtual ~TaskClient();

  typedef std_msgs::msg::String Goal;
  typedef std_msgs::msg::String GoalID;
  typedef std_msgs::msg::String Result;
  typedef std_msgs::msg::String Feedback;
  typedef std_msgs::msg::String Status;

  void execute();
  void cancel();

protected:
  void onResultReceived(const Result::SharedPtr msg);
  void onFeedbackReceived(const Feedback::SharedPtr msg);
  void onStatusReceived(const Status::SharedPtr msg);

  rclcpp::Node * node_;

  rclcpp::Publisher<Goal>::SharedPtr goalPub_;
  rclcpp::Publisher<GoalID>::SharedPtr cancelPub_;

  rclcpp::Subscription<Result>::SharedPtr resultSub_;
  rclcpp::Subscription<Feedback>::SharedPtr feedbackSub_;
  rclcpp::Subscription<Status>::SharedPtr statusSub_;
};

#endif  // TASK__TASKCLIENT_HPP_
