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

#include <memory>

#include "nav2_navfn_planner/navfn_planner.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  using namespace rclcpp::executors;
  using namespace rclcpp::executor;

  rclcpp::init(argc, argv);

  auto planner = std::make_shared<nav2_navfn_planner::NavfnPlanner>();

  // The planner needs a special executor
  // <FEEDBACK REQUEST> I'm open to suggestions but I'd rather not expose we're using
  // a multithreaded executor due to some implementation detail. Can you think of a better pattern?
  // Also consider we'll have to revist this once we start composing nodes.
  auto exec = planner->get_executor();
  exec->spin();

  rclcpp::shutdown();
  return 0;
}
