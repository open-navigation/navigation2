// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"

class FollowPathActionServer : public TestActionServer<nav2_msgs::action::FollowPath>
{
public:
  FollowPathActionServer()
  : TestActionServer("follow_path")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::FollowPath>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::FollowPath::Result>();
    goal_handle->succeed(result);
  }
};

class FollowPathActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("follow_path_action_test_fixture");
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

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::FollowPathAction>(
          name, "follow_path", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::FollowPathAction>(
      "FollowPath", builder);
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

  static std::shared_ptr<FollowPathActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr FollowPathActionTestFixture::node_ = nullptr;
std::shared_ptr<FollowPathActionServer>
FollowPathActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * FollowPathActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> FollowPathActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> FollowPathActionTestFixture::tree_ = nullptr;

TEST_F(FollowPathActionTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <FollowPath path="{path}" controller_id="FollowPath"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // set new path on blackboard
  nav_msgs::msg::Path path;
  path.poses.resize(1);
  path.poses[0].pose.position.x = 1.0;
  config_->blackboard->set("path", path);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // the goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("controller_id"), std::string("FollowPath"));
  EXPECT_EQ(action_server_->getCurrentGoal()->path.poses.size(), 1u);
  EXPECT_EQ(action_server_->getCurrentGoal()->path.poses[0].pose.position.x, 1.0);
  EXPECT_EQ(action_server_->getCurrentGoal()->controller_id, std::string("FollowPath"));

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);

  // set new goal
  path.poses[0].pose.position.x = -2.5;
  config_->blackboard->set("path", path);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->path.poses.size(), 1u);
  EXPECT_EQ(action_server_->getCurrentGoal()->path.poses[0].pose.position.x, -2.5);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  FollowPathActionTestFixture::action_server_ =
    std::make_shared<FollowPathActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(FollowPathActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
