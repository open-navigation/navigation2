// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
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

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/smoother_selector_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"

class SmootherSelectorTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("smoother_selector_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);

    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<nav2_behavior_tree::SmootherSelector>(name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::SmootherSelector>(
      "SmootherSelector",
      builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr SmootherSelectorTestFixture::node_ = nullptr;

BT::NodeConfiguration * SmootherSelectorTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> SmootherSelectorTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> SmootherSelectorTestFixture::tree_ = nullptr;

TEST_F(SmootherSelectorTestFixture, test_custom_topic)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <SmootherSelector selected_smoother="{selected_smoother}" default_smoother="DWB" topic_name="smoother_selector_custom_topic_name"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // check default value
  std::string selected_smoother_result;
  EXPECT_TRUE(config_->blackboard->get("selected_smoother", selected_smoother_result));

  EXPECT_EQ(selected_smoother_result, "DWB");

  std_msgs::msg::String selected_smoother_cmd;

  selected_smoother_cmd.data = "DWC";

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  auto smoother_selector_pub =
    node_->create_publisher<std_msgs::msg::String>("smoother_selector_custom_topic_name", qos);

  // publish a few updates of the selected_smoother
  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    smoother_selector_pub->publish(selected_smoother_cmd);

    rclcpp::spin_some(node_);
  }

  // check smoother updated
  EXPECT_TRUE(config_->blackboard->get("selected_smoother", selected_smoother_result));
  EXPECT_EQ("DWC", selected_smoother_result);
}

TEST_F(SmootherSelectorTestFixture, test_default_topic)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <SmootherSelector selected_smoother="{selected_smoother}" default_smoother="GridBased"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // check default value
  std::string selected_smoother_result;
  EXPECT_TRUE(config_->blackboard->get("selected_smoother", selected_smoother_result));

  EXPECT_EQ(selected_smoother_result, "GridBased");

  std_msgs::msg::String selected_smoother_cmd;

  selected_smoother_cmd.data = "RRT";

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  auto smoother_selector_pub =
    node_->create_publisher<std_msgs::msg::String>("smoother_selector", qos);

  // publish a few updates of the selected_smoother
  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    smoother_selector_pub->publish(selected_smoother_cmd);

    rclcpp::spin_some(node_);
  }

  // check smoother updated
  EXPECT_TRUE(config_->blackboard->get("selected_smoother", selected_smoother_result));
  EXPECT_EQ("RRT", selected_smoother_result);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
