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
#include "global_planners/start_occupied_planner.hpp"
#include "global_planners/goal_occupied_planner.hpp"
#include "global_planners/start_outside_map_planner.hpp"
#include "global_planners/goal_outside_map_planner.hpp"
#include "global_planners/no_valid_path_planner.hpp"
#include "global_planners/tf_error_planner.hpp"
#include "global_planners/timed_out_planner.hpp"
#include "global_planners/no_via_points_given_planner.hpp"

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::StartOccupied, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::GoalOccupied, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::StartOutsideMapBounds, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::GoalOccupied, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::GoalOutsideMapBounds, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::NoValidPathCouldBeFound, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::PlannerTFError, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::PlannerTimedOut, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::NoViapointsGiven, nav2_core::GlobalPlanner)
