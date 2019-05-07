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

#include <vector>
#include <string>
#include <algorithm>
#include <memory>

#include "nav2_costmap_2d/clear_costmap_service.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_costmap_2d
{

using std::vector;
using std::string;
using std::shared_ptr;
using std::any_of;
using ClearExceptRegion = nav2_msgs::srv::ClearCostmapExceptRegion;
using ClearEntirely = nav2_msgs::srv::ClearEntireCostmap;

ClearCostmapService::ClearCostmapService(
  nav2_lifecycle::LifecycleNode::SharedPtr node,
  Costmap2DROS & costmap)
: node_(node), costmap_(costmap)
{
  reset_value_ = costmap_.getCostmap()->getDefaultValue();

  std::vector<std::string> clearable_layers{"obstacle_layer"};
  node_->declare_parameter("clearable_layers", rclcpp::ParameterValue(clearable_layers));

  node_->get_parameter("clearable_layers", clearable_layers_);

  clear_except_service_ = node_->create_service<ClearExceptRegion>(
    "clear_except_" + costmap_.getName(),
    std::bind(&ClearCostmapService::clearExceptRegionCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  clear_entire_service_ = node_->create_service<ClearEntirely>(
    "clear_entirely_" + costmap_.getName(),
    std::bind(&ClearCostmapService::clearEntireCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void ClearCostmapService::clearExceptRegionCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearExceptRegion::Request> request,
  const shared_ptr<ClearExceptRegion::Response>/*response*/)
{
  RCLCPP_INFO(node_->get_logger(),
    "Received request to clear except a region the " + costmap_.getName());

  clearExceptRegion(request->reset_distance);
}

void ClearCostmapService::clearEntireCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ClearEntirely::Request>/*request*/,
  const std::shared_ptr<ClearEntirely::Response>/*response*/)
{
  RCLCPP_INFO(node_->get_logger(), "Received request to clear entirely the " + costmap_.getName());

  clearEntirely();
}

void ClearCostmapService::clearExceptRegion(const double reset_distance)
{
  double x, y;

  if (!getPosition(x, y)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot clear map because robot pose cannot be retrieved.");
    return;
  }

  auto layers = costmap_.getLayeredCostmap()->getPlugins();

  for (auto & layer : *layers) {
    if (isClearable(getLayerName(*layer))) {
      auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
      clearLayerExceptRegion(costmap_layer, x, y, reset_distance);
    }
  }
}

void ClearCostmapService::clearEntirely()
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getCostmap()->getMutex()));
  costmap_.resetLayers();
}

bool ClearCostmapService::isClearable(const string & layer_name) const
{
  return count(begin(clearable_layers_), end(clearable_layers_), layer_name) != 0;
}

void ClearCostmapService::clearLayerExceptRegion(
  shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance)
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  double start_point_x = pose_x - reset_distance / 2;
  double start_point_y = pose_y - reset_distance / 2;
  double end_point_x = start_point_x + reset_distance;
  double end_point_y = start_point_y + reset_distance;

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();

  // Clearing the four rectangular regions around the one we want to keep
  // top region
  costmap->resetMapToValue(0, 0, size_x, start_y, reset_value_);
  // left region
  costmap->resetMapToValue(0, start_y, start_x, end_y, reset_value_);
  // right region
  costmap->resetMapToValue(end_x, start_y, size_x, end_y, reset_value_);
  // bottom region
  costmap->resetMapToValue(0, end_y, size_x, size_y, reset_value_);

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
}

bool ClearCostmapService::getPosition(double & x, double & y) const
{
  geometry_msgs::msg::PoseStamped pose;
  if (!costmap_.getRobotPose(pose)) {
    return false;
  }

  x = pose.pose.position.x;
  y = pose.pose.position.y;

  return true;
}

string ClearCostmapService::getLayerName(const Layer & layer) const
{
  string name = layer.getName();

  int slash = name.rfind('/');

  if (slash != std::string::npos) {
    name = name.substr(slash + 1);
  }

  return name;
}

}  // namespace nav2_costmap_2d
