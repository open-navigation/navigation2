// Copyright (c) 2020 Shivamm Pandey pandeyshivam2017robotics@gmail.com
// Copyright (c) 2020 Samsung Research Russia
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

//
// Created by shivam on 10/11/20.
//

#include <gtest/gtest.h>
#include <experimental/filesystem>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <vector>

#include "test_constants/test_constants.h"
#include "nav2_map_server/map_saver.hpp"
#include "nav2_util/lifecycle_service_client.hpp"

#include "nav2_msgs/srv/save_map3_d.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"

#define TEST_DIR TEST_DIRECTORY

using std::experimental::filesystem::path;
using lifecycle_msgs::msg::Transition;
using namespace nav2_map_server;  // NOLINT

class RclCppFixture
{
 public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

class MapSaverTestFixture : public ::testing::Test
{
 public:
  static void SetUpTestCase()
  {
    node_ = rclcpp::Node::make_shared("map_client_test");
    lifecycle_client_ =
        std::make_shared<nav2_util::LifecycleServiceClient>("map_saver", node_);
    RCLCPP_INFO(node_->get_logger(), "Creating Test Node");

    std::this_thread::sleep_for(std::chrono::seconds(5));  // allow node to start up
    const std::chrono::seconds timeout(5);
    lifecycle_client_->change_state(Transition::TRANSITION_CONFIGURE, timeout);
    lifecycle_client_->change_state(Transition::TRANSITION_ACTIVATE, timeout);
  }

  static void TearDownTestCase()
  {
    lifecycle_client_->change_state(Transition::TRANSITION_DEACTIVATE);
    lifecycle_client_->change_state(Transition::TRANSITION_CLEANUP);
  }

  template<class T>
  typename T::Response::SharedPtr send_request(

      rclcpp::Node::SharedPtr node,
      typename rclcpp::Client<T>::SharedPtr client,
      typename T::Request::SharedPtr request)
  {
    auto result = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      return result.get();
    } else {
      return nullptr;
    }
  }

 protected:

  static void verifyMapMsg(
      const sensor_msgs::msg::PointCloud2 & map_msg,
      const geometry_msgs::msg::Pose & origin)
  {
    std::vector<float> center;
    center.push_back(origin.position.x);
    center.push_back(origin.position.y);
    center.push_back(origin.position.z);

    std::vector<float> orientation;
    orientation.push_back(origin.orientation.w);
    orientation.push_back(origin.orientation.x);
    orientation.push_back(origin.orientation.y);
    orientation.push_back(origin.orientation.z);
    ASSERT_EQ(center, g_valid_center_pcd);
    ASSERT_EQ(orientation, g_valid_orientation_pcd);

    ASSERT_EQ(map_msg.width, g_valid_pcd_width);
    ASSERT_EQ(map_msg.data.size(), g_valid_pcd_data_size);
  }

  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<nav2_util::LifecycleServiceClient> lifecycle_client_;
};

rclcpp::Node::SharedPtr MapSaverTestFixture::node_ = nullptr;
std::shared_ptr<nav2_util::LifecycleServiceClient> MapSaverTestFixture::lifecycle_client_ =
    nullptr;

// Send map(pcd) saving service request.
// Load saved map and verify obtained OccupancyGrid.
TEST_F(MapSaverTestFixture, SaveMap3D)
{
  RCLCPP_INFO(node_->get_logger(), "Testing SaveMap service");
  auto req = std::make_shared<nav2_msgs::srv::SaveMap3D::Request>();
  auto client = node_->create_client<nav2_msgs::srv::SaveMap3D>(
      "/map_saver/save_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for save_map service");
  ASSERT_TRUE(client->wait_for_service());

  // 1. Send valid save_map service request
  req->map_topic = "map";
  req->map_url = path(g_tmp_dir) / path(g_valid_pcd_map_name);

  // set view point translation
  req->origin.position.x = g_valid_center_pcd[0];
  req->origin.position.y = g_valid_center_pcd[1];
  req->origin.position.z = g_valid_center_pcd[2];

  // set view point rotation
  req->origin.orientation.w = g_valid_orientation_pcd[0];
  req->origin.orientation.x = g_valid_orientation_pcd[1];
  req->origin.orientation.y = g_valid_orientation_pcd[2];
  req->origin.orientation.z = g_valid_orientation_pcd[3];

  req->file_format = "pcd";
  req->as_binary = false;

  auto resp = send_request<nav2_msgs::srv::SaveMap3D>(node_, client, req);
  ASSERT_EQ(resp->result, true);

  // 2. Load saved map and verify it
  sensor_msgs::msg::PointCloud2 map_msg;
  geometry_msgs::msg::Pose pose_msg;

  map_3d::LOAD_MAP_STATUS status =
      map_3d::loadMapFromYaml(
          path(g_tmp_dir) / path(g_valid_pcd_yaml_file),
          map_msg, pose_msg);

  ASSERT_EQ(status, map_3d::LOAD_MAP_SUCCESS);
  verifyMapMsg(map_msg, pose_msg);
}

// Send map(pcd) saving service request with default parameters.
// Load saved map(pcd) and verify obtained PointCloud.
TEST_F(MapSaverTestFixture, SaveMapDefaultParameters3D)
{
  RCLCPP_INFO(node_->get_logger(), "Testing SaveMap service");
  auto req = std::make_shared<nav2_msgs::srv::SaveMap3D::Request>();
  auto client = node_->create_client<nav2_msgs::srv::SaveMap3D>(
      "/map_saver/save_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for save_map service");
  ASSERT_TRUE(client->wait_for_service());

  // 1. Send save_map service request with default parameters
  req->map_topic = "";
  req->map_url = path(g_tmp_dir) / path(g_valid_pcd_map_name);

  // set view point translation
  req->origin.position.x = g_valid_center_pcd[0];
  req->origin.position.y = g_valid_center_pcd[1];
  req->origin.position.z = g_valid_center_pcd[2];

  // set view point rotation
  req->origin.orientation.w = g_valid_orientation_pcd[0];
  req->origin.orientation.x = g_valid_orientation_pcd[1];
  req->origin.orientation.y = g_valid_orientation_pcd[2];
  req->origin.orientation.z = g_valid_orientation_pcd[3];

  req->file_format = "";
  req->as_binary = false;

  auto resp = send_request<nav2_msgs::srv::SaveMap3D>(node_, client, req);
  ASSERT_EQ(resp->result, true);

  // 2. Load saved map and verify it
  sensor_msgs::msg::PointCloud2 map_msg;
  geometry_msgs::msg::Pose pose_msg;
  map_3d::LOAD_MAP_STATUS status =
      map_3d::loadMapFromYaml(
          path(g_tmp_dir) / path(g_valid_pcd_yaml_file),
          map_msg, pose_msg);

  ASSERT_EQ(status, map_3d::LOAD_MAP_SUCCESS);
  verifyMapMsg(map_msg, pose_msg);
}
