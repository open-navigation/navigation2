// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2021 Samsung Research America
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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/navigate_through_poses_action.hpp"

class NavigateThroughPosesActionServer
  : public TestActionServer<nav2_msgs::action::NavigateThroughPoses>
{
public:
  NavigateThroughPosesActionServer()
  : TestActionServer("navigate_through_poses")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateThroughPoses>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::NavigateThroughPoses::Result>();
    goal_handle->succeed(result);
  }
};

class NavigateThroughPosesActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("navigate_through_poses_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout",
      std::chrono::milliseconds(1000));
    config_->blackboard->set("initial_pose_received", false);
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    config_->blackboard->set(
      "goals", poses);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::NavigateThroughPosesAction>(
          name, "navigate_through_poses", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::NavigateThroughPosesAction>(
      "NavigateThroughPoses", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<NavigateThroughPosesActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr NavigateThroughPosesActionTestFixture::node_ = nullptr;
std::shared_ptr<NavigateThroughPosesActionServer>
NavigateThroughPosesActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * NavigateThroughPosesActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> NavigateThroughPosesActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> NavigateThroughPosesActionTestFixture::tree_ = nullptr;

TEST_F(NavigateThroughPosesActionTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <NavigateThroughPoses goals="{goals}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.resize(1);
  poses[0].pose.position.x = -2.5;
  poses[0].pose.orientation.x = 1.0;
  config_->blackboard->set("goals", poses);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->poses, poses);

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  NavigateThroughPosesActionTestFixture::action_server_ =
    std::make_shared<NavigateThroughPosesActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(NavigateThroughPosesActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
