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

#include <memory>
#include <filesystem>

#include "nav2_route/graph_file_loader.hpp"

// TODO(jw) move into plugin
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>

namespace nav2_route
{

GraphFileLoader::GraphFileLoader(
  nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame)
{
  // TODO(sm) plugin header + loader to use instead. THis will manage plugin get / load
  // TODO(sm) logging
  tf_ = tf;
  route_frame_ = frame;
  (void)node;
  nav2_util::declare_parameter_if_not_declared(
    node, "graph_filepath", rclcpp::ParameterValue(std::string("hi!")));
  // TODO(sm) rclcpp::ParameterType::PARAMETER_STRING
  graph_filepath_ = node->get_parameter("graph_filepath").as_string();
}

bool GraphFileLoader::loadGraphFromFile(Graph &graph, GraphToIDMap &, std::string)
{
  // Check filepath exists TODO(sm)
//  std::string filepath_to_load;
//  if (filepath.empty()) {
//    filepath_to_load = graph_filepath_;
//  } else {
//    filepath_to_load = filepath;
//  }

  // if (!fileExists(filepath_to_load)) {
  //   RCLCPP_ERROR(node->get_logger(), "Graph file %s does not exist!", graph_filename_);
  //   return false;
  // }

  // Validate file is legit using a plugin API TODO(sm)
  // Load file using a plugin API


  // Convert all coordinates to `frame` (in a new method) for standardization
  // Including conversion of GPS coordinates, so we can populate it in some
  // cartesian frame necessary for traversal cost estimation and densifying
  // (and so we don't need to propogate it through our structures)


//  idx_map.clear();
//  graph.clear();
//
//  // A test graph for visualization and prototyping
//  graph.resize(9);
//  unsigned int idx = 0;
//  unsigned int ids = 1;
//  for (unsigned int i = 0; i != 3; i++) {
//    for (unsigned int j = 0; j != 3; j++) {
//      graph[idx].nodeid = ids;
//      graph[idx].coords.x = i;
//      graph[idx].coords.y = j;
//      idx_map[graph[idx].nodeid] = idx;  // for a nodeid key, provide the graph idx value
//      idx++;
//      ids++;
//    }
//  }
//
//  EdgeCost default_cost;
//  // Creates bidirectional routs from 0-1-4-5 & directional only route from 0-3-6
//  graph[0].addEdge(default_cost, &graph[1], ids++);
//  graph[1].addEdge(default_cost, &graph[0], ids++);
//  graph[4].addEdge(default_cost, &graph[1], ids++);
//  graph[1].addEdge(default_cost, &graph[4], ids++);
//  graph[5].addEdge(default_cost, &graph[4], ids++);
//  graph[4].addEdge(default_cost, &graph[5], ids++);
//  graph[0].addEdge(default_cost, &graph[3], ids++);
//  graph[3].addEdge(default_cost, &graph[6], ids++);

  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_route");
  std::string file_path = pkg_share_dir + "/graphs/geojson/aws_graph.geojson";

  std::ifstream graph_file(file_path);

  json json;
  graph_file >> json;

  auto features = json.at("features");

  std::vector<nlohmann::json> nodes;
  std::vector<nlohmann::json> edges;
  getNodes(features, nodes);
  getEdges(features, edges);

  graph.resize(nodes.size());
  addNodesToGraph(graph, nodes);
  addEdgesToGraph(graph, edges);

  return true;
}

bool GraphFileLoader::fileExists(const std::string & filepath)
{
  return std::filesystem::exists(filepath);
}

void GraphFileLoader::getNodes(const json & features, std::vector<json> & nodes)
{
  for (const auto & feature : features) {
    if (feature["geometry"]["type"] == "Point") {
      nodes.emplace_back(feature);
    }
  }
}

void GraphFileLoader::getEdges(const json & features, std::vector<json> & edges)
{
  for (const auto & feature : features) {
    if (feature["geometry"]["type"] == "MultiLineString") {
      edges.emplace_back(feature);
    }
  }
}

void GraphFileLoader::addNodesToGraph(nav2_route::Graph & graph, std::vector<json> & nodes)
{
  for (const auto & node : nodes) {
    // Required data
    unsigned int id = node["properties"]["id"];
    float x = node["geometry"]["coordinates"][0];
    float y = node["geometry"]["coordinates"][1];
    graph[id].nodeid = id;
    graph[id].coords.x = x;
    graph[id].coords.y = y;
    // graph[id].coords.frame_id = frame; // TODO(jw) use default for now
  }
}

void GraphFileLoader::addEdgesToGraph(nav2_route::Graph & graph, std::vector<json> & edges)
{
  nav2_route::EdgeCost edge_cost;
  for (const auto & edge : edges) {
    // Required data
    const auto edge_properties = edge["properties"];
    unsigned int id = edge_properties["id"];
    unsigned int start_id = edge_properties["startid"];
    unsigned int end_id = edge_properties["endid"];

    // Recommended data
    if ( edge_properties.contains("cost"))
    {
      edge_cost.cost = edge_properties["cost"];
    }

    if ( edge_properties.contains("overridable"))
    {
      edge_cost.overridable = edge_properties["overridable"];
    }

    graph[start_id].addEdge(edge_cost, &graph[end_id], id);
  }
}


}  // namespace nav2_route
