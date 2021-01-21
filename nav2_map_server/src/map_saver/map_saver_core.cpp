// Copyright (c) 2020 Shivam Pandey pandeyshivam2017robotics@gmail.com
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

#include "nav2_map_server/map_saver_core.hpp"

namespace nav2_map_server
{

template<class mapT>
MapSaver<mapT>::MapSaver()
: nav2_util::LifecycleNode("map_saver")
{
}

template<class mapT>
MapSaver<mapT>::~MapSaver() = default;

template<class mapT>
nav2_util::CallbackReturn
MapSaver<mapT>::on_configure(const rclcpp_lifecycle::State & state)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapSaver<mapT>::on_activate(const rclcpp_lifecycle::State & state)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapSaver<mapT>::on_deactivate(const rclcpp_lifecycle::State & state)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapSaver<mapT>::on_cleanup(const rclcpp_lifecycle::State & state)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapSaver<mapT>::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace nav2_map_server
