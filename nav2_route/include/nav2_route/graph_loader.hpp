// Copyright (c) 2023, Samsung Research America
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

#ifndef NAV2_ROUTE__GRAPH_LOADER_HPP_
#define NAV2_ROUTE__GRAPH_LOADER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>

#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_route/types.hpp"
#include "nav2_route/interfaces/graph_file_loader.hpp"

namespace nav2_route
{

/**
 * @class nav2_route::GraphLoader
 * @brief An action server to load graph files into the graph object for search and processing
 */
class GraphLoader
{
public:
  /**
   * @brief A constructor for nav2_route::GraphLoader
   * @param options Additional options to control creation of the node.
   */
  explicit GraphLoader(
    nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame);

  /**
   * @brief A destructor for nav2_route::GraphLoader
   */
  ~GraphLoader() = default;

  using GraphFileLoaderMap = std::unordered_map<std::string, GraphFileLoader::Ptr>;

  /**
   * @brief Loads a graph object with file information
   * @param graph Graph to populate
   * @param idx_map A map translating nodeid's to graph idxs for use in graph modification
   * services and idx-based route planning requests. This is much faster than using a
   * map the full graph data structure.
   * @param filepath The filepath to the graph data
   * @param graph_file_loader_id The id of the GraphFileLoader
   * @return bool If successful
   */
  bool loadGraphFromFile(
    Graph & graph,
    GraphToIDMap & idx_map,
    std::string filepath = "",
    std::string graph_file_loader_id = "");

protected:
  std::string route_frame_, graph_filepath_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("GraphLoader")};

  // Graph Parser
  pluginlib::ClassLoader<GraphFileLoader> plugin_loader_;
  GraphFileLoaderMap graph_file_loader_;
  std::string default_plugin_id_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__GRAPH_LOADER_HPP_
