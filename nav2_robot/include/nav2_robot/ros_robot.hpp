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

#ifndef NAV2_ROBOT__ROS_ROBOT_HPP_
#define NAV2_ROBOT__ROS_ROBOT_HPP_

#include <string>
#include "nav2_robot/robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nav2_robot
{

class RosRobot : public Robot
{
public:
  using Footprint = std::vector<geometry_msgs::msg::Point>;
  explicit RosRobot(rclcpp::Node * node);
  RosRobot() = delete;
  ~RosRobot();

  void enterSafeState() override;

  bool getCurrentPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose);
  bool getCurrentVelocity(nav_msgs::msg::Odometry::SharedPtr & robot_velocity);
  Footprint getFootprint();

  std::string getRobotName();
  void sendVelocity(geometry_msgs::msg::Twist twist);

protected:
  rclcpp::Node * node_;
  std::string urdf_file_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
  nav_msgs::msg::Odometry::SharedPtr current_velocity_;

  bool initial_pose_received_;
  bool initial_odom_received_;
  urdf::Model model_;

  void onPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace nav2_robot

#endif  // NAV2_ROBOT__ROS_ROBOT_HPP_
