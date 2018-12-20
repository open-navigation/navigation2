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

#ifndef NAV2_WORLD_MODEL__WORLD_REPRESENTATION_HPP_
#define NAV2_WORLD_MODEL__WORLD_REPRESENTATION_HPP_

#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/srv/process_region.hpp"

namespace nav2_world_model
{

using nav2_msgs::srv::GetCostmap;
using nav2_msgs::srv::ProcessRegion;

class WorldRepresentation
{
public:
  virtual GetCostmap::Response getCostmap(const GetCostmap::Request & request) = 0;

  // Verify if a region is unoccupied
  virtual ProcessRegion::Response confirmFreeSpace(const ProcessRegion::Request & request) = 0;

  // Request to set a region as unoccupied
  virtual ProcessRegion::Response clearArea(const ProcessRegion::Request & request) = 0;

  virtual ~WorldRepresentation() {}
};

}  // namespace nav2_world_model

#endif  // NAV2_WORLD_MODEL__WORLD_REPRESENTATION_HPP_
