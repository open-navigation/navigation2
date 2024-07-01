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
// limitations under the License.

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/assisted_teleop_cancel_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

class CancelAssistedTeleopServer : public TestActionServer<nav2_msgs::action::AssistedTeleop>
{
public:
  CancelAssistedTeleopServer()
  : TestActionServer("assisted_teleop")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::AssistedTeleop>>
    goal_handle)
  {
    while (!goal_handle->is_canceling()) {
      // Assisted Teleop here until goal cancels
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
  }
};

class CancelAssistedTeleopActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("cancel_back_up_action_test_fixture");
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
    client_ = rclcpp_action::create_client<nav2_msgs::action::AssistedTeleop>(
      node_, "assisted_teleop");

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::AssistedTeleopCancel>(
          name, "assisted_teleop", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::AssistedTeleopCancel>(
      "CancelAssistedTeleop", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    client_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<CancelAssistedTeleopServer> action_server_;
  static std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::AssistedTeleop>> client_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr CancelAssistedTeleopActionTestFixture::node_ = nullptr;
std::shared_ptr<CancelAssistedTeleopServer>
CancelAssistedTeleopActionTestFixture::action_server_ = nullptr;
std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::AssistedTeleop>>
CancelAssistedTeleopActionTestFixture::client_ = nullptr;

BT::NodeConfiguration * CancelAssistedTeleopActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory>
CancelAssistedTeleopActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> CancelAssistedTeleopActionTestFixture::tree_ = nullptr;

TEST_F(CancelAssistedTeleopActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
             <CancelAssistedTeleop name="AssistedTeleopCancel"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  auto send_goal_options = rclcpp_action::Client<
    nav2_msgs::action::AssistedTeleop>::SendGoalOptions();

  // Creating a dummy goal_msg
  auto goal_msg = nav2_msgs::action::AssistedTeleop::Goal();

  // BackUping for server and sending a goal
  client_->wait_for_action_server();
  client_->async_send_goal(goal_msg, send_goal_options);

  // Adding a sleep so that the goal is indeed older than 10ms as described in our abstract class
  std::this_thread::sleep_for(std::chrono::milliseconds(15));

  // Executing tick
  tree_->rootNode()->executeTick();

  // BT node should return success, once when the goal is cancelled
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  // Adding another test case to check if the goal is infact cancelling
  EXPECT_EQ(action_server_->isGoalCancelled(), true);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and back_up on new thread
  CancelAssistedTeleopActionTestFixture::action_server_ =
    std::make_shared<CancelAssistedTeleopServer>();
  std::thread server_thread([]() {
      rclcpp::spin(CancelAssistedTeleopActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
