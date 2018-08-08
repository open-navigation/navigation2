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

#ifndef PLANNING__DIJKSTRAPLANNER_HPP_
#define PLANNING__DIJKSTRAPLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include "planning/ComputePathToPoseTaskServer.hpp"
#include "planning/Navfn.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"

class DijkstraPlanner : public ComputePathToPoseTaskServer
{
public:
  explicit DijkstraPlanner(const std::string & name);
  DijkstraPlanner() = delete;
  ~DijkstraPlanner();

  TaskStatus executeAsync(const ComputePathToPoseCommand::SharedPtr command) override;

private:
  // Compute a plan given start and goal poses, provided in global world frame.
  bool makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, 
    double tolerance, std::vector<geometry_msgs::msg::PoseStamped>& plan);

  // Compute the navigation function given a seed point in the world to start from
  bool computePotential(const geometry_msgs::msg::Point& world_point);

  // Compute a plan to a goal from a potential - must call computePotential first
  bool getPlanFromPotential(const geometry_msgs::msg::PoseStamped& goal, std::vector<geometry_msgs::msg::PoseStamped>& plan);

  // Compute the potential, or navigation cost, at a given point in the world - must call computePotential first
  double getPointPotential(const geometry_msgs::msg::Point& world_point);

  // Check for a valid potential value at a given point in the world - must call computePotential first
  bool validPointPotential(const geometry_msgs::msg::Point& world_point);
  bool validPointPotential(const geometry_msgs::msg::Point& world_point, double tolerance);

  // Compute the squared distance between two points
  inline double squared_distance(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2)
  {
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return dx*dx + dy*dy;
  }

  // Transform a point from world to map frame
  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);

  // Transform a point from map to world frame
  void mapToWorld(double mx, double my, double& wx, double& wy);

  // Set the corresponding cell cost to be free space
  void clearRobotCell(unsigned int mx, unsigned int my);

  // Request costmap from world model
  void getCostmap(nav2_msgs::msg::Costmap& costmap, const std::chrono::milliseconds waitTime = std::chrono::milliseconds(100));

  // Wait for costmap server to appear
  bool waitForCostmapServer(const std::chrono::seconds waitTime = std::chrono::seconds(10));

  // Publish plan for visualization purposes
  void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path);

  // Planner based on ROS1 NavFn algorithm
  std::shared_ptr<NavFn> planner_;

  // Client for getting the costmap
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;

  // Computed path publisher
  rclcpp::Publisher<nav2_msgs::msg::Path>::SharedPtr plan_publisher_;

  // The costmap to use
  nav2_msgs::msg::Costmap costmap_;

  // The global frame of the costmap
  std::string global_frame_;

  // Whether or not the planner should be allowed to plan through unknown space
  bool allow_unknown_;

  // Amount the planner can relax the space constraint
  double default_tolerance_;
};

#endif  // PLANNING__DIJKSTRAPLANNER_HPP_
