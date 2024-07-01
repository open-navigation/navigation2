// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/truncate_path_action.hpp"


class TruncatePathTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("change_goal_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set(
      "node",
      node_);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::TruncatePath>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::TruncatePath>(
      "TruncatePath", builder);
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

rclcpp::Node::SharedPtr TruncatePathTestFixture::node_ = nullptr;

BT::NodeConfiguration * TruncatePathTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> TruncatePathTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> TruncatePathTestFixture::tree_ = nullptr;

TEST_F(TruncatePathTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <TruncatePath distance="1.0" input_path="{path}" output_path="{truncated_path}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.0);
  path.poses.push_back(pose);

  pose.pose.position.x = 0.5;
  pose.pose.position.y = 0.0;
  path.poses.push_back(pose);

  pose.pose.position.x = 0.9;
  pose.pose.position.y = 0.0;
  path.poses.push_back(pose);

  pose.pose.position.x = 1.5;
  pose.pose.position.y = 0.5;
  path.poses.push_back(pose);

  EXPECT_EQ(path.poses.size(), 4u);

  config_->blackboard->set("path", path);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  nav_msgs::msg::Path truncated_path;
  EXPECT_TRUE(config_->blackboard->get("truncated_path", truncated_path));

  EXPECT_NE(path, truncated_path);
  EXPECT_EQ(truncated_path.poses.size(), 2u);

  double r, p, y;
  tf2::Quaternion q;
  tf2::fromMsg(truncated_path.poses.back().pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);

  EXPECT_NEAR(y, 0.463, 0.001);
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
