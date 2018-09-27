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
// limitations under the License. Reserved.

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "planner_tester.hpp"

using namespace std::chrono_literals;
using nav2_util::PlannerTester;
using nav2_util::TestCostmap;

// rclcpp::init can only be called once per process, so this needs to be a global variable
class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

void printPath(
  std::shared_ptr<nav2_tasks::ComputePathToPoseResult> path,
  std::shared_ptr<rclcpp::Node> node)
{
  int index = 0;
  for (auto pose : path->poses) {
    RCLCPP_INFO(node->get_logger(), "  point %u x: %0.2f, y: %0.2f",
      index, pose.position.x, pose.position.y);
    ++index;
  }
}

TEST_F(PlannerTester, testSimpleCostmaps)
{
  std::vector<TestCostmap> costmaps = {
    TestCostmap::open_space,
    TestCostmap::bounded,
    TestCostmap::top_left_obstacle,
    TestCostmap::bottom_left_obstacle,
    TestCostmap::maze1,
    TestCostmap::maze2
  };

  auto result = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

  for (auto costmap : costmaps) {
    loadSimpleCostmap(costmap);
    EXPECT_EQ(true, defaultPlannerTest(result));
  }
}

TEST_F(PlannerTester, testWithOneFixedEndpoint)
{
  loadMap();
  auto result = std::make_shared<nav2_tasks::ComputePathToPoseResult>();
  EXPECT_EQ(true, defaultPlannerTest(result));
}

// TODO(orduno): refine a bit more this test
//               for example, check the output after each point, not only after the whole batch
TEST_F(PlannerTester, testWithThousandRandomEndPoints)
{
  loadMap();
  auto result = std::make_shared<nav2_tasks::ComputePathToPoseResult>();
  EXPECT_EQ(true, defaultPlannerRandomTests(1000));
}
