// Copyright (c) 2022 Joshua Wallace
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
// limitations under the License. Reserved.

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "nav2_msgs/srv/is_path_valid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "planner_tester.hpp"
#include "nav2_util/lifecycle_utils.hpp"

using nav2_system_tests::PlannerTester;
using nav2_util::TestCostmap;

TEST(testIsPathValid, testIsPathValid)
{
  auto planner_tester = std::make_shared<PlannerTester>();
  planner_tester->activate();
  // load open space cost map which is 10 by 10
  planner_tester->loadSimpleCostmap(TestCostmap::open_space);

  // create a fake service request
  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  nav_msgs::msg::Path path;
  for(int i = 1; i < 10; ++i)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    pose.pose.position.y = i;
    request->path.poses.push_back(pose);
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("path_valid_node");
  auto client = node->create_client<nav2_msgs::srv::IsPathValid>("/is_path_valid");

  auto result = client->async_send_request(request);

  while(!client->wait_for_service(std::chrono::milliseconds(1000)))
  {
    RCLCPP_INFO(node->get_logger(), "Waiting for service");
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for service complete");
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Got result");
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to call service: ");
  }

  EXPECT_EQ(0,0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  auto planner_tester = std::make_shared<PlannerTester>();

  planner_tester->activate();

  // load open space cost map which is 10 by 10
  planner_tester->loadSimpleCostmap(TestCostmap::open_space);

  bool all_successful = RUN_ALL_TESTS();

  // launch the planner_tester on a new thread
  std::thread planner_thread([]() {
    auto planner_tester = std::make_shared<PlannerTester>();
    planner_tester->activate();
    // load open space cost map which is 10 by 10
    planner_tester->loadSimpleCostmap(TestCostmap::open_space);
    rclcpp::spin(planner_tester->get_node_base_interface());
  });

  // shutdown ROS
  rclcpp::shutdown();
  planner_thread.join();
  return all_successful;


}
