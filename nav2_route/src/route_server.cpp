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
// limitations under the License. Reserved.

#include "nav2_route/route_server.hpp"

using nav2_util::declare_parameter_if_not_declared;
using std::placeholders::_1;
using std::placeholders::_2;

namespace nav2_route
{

RouteServer::RouteServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("route_server", "", options)
{}

nav2_util::CallbackReturn
RouteServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  auto node = shared_from_this();
  graph_vis_publisher_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "route_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  compute_route_server_ = std::make_shared<ComputeRouteServer>(
    node, "compute_route",
    std::bind(&RouteServer::computeRoute, this),
    nullptr, std::chrono::milliseconds(500), true);

  compute_and_track_route_server_ = std::make_shared<ComputeAndTrackRouteServer>(
    node, "compute_and_track_route",
    std::bind(&RouteServer::computeAndTrackRoute, this),
    nullptr, std::chrono::milliseconds(500), true);

  set_graph_service_ = node->create_service<nav2_msgs::srv::SetRouteGraph>(
    "set_route_graph",
    std::bind(
      &RouteServer::setRouteGraph, this,
      std::placeholders::_1, std::placeholders::_2));

  declare_parameter_if_not_declared(
    node, "route_frame", rclcpp::ParameterValue(std::string("map")));
  declare_parameter_if_not_declared(
    node, "base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter_if_not_declared(
    node, "max_planning_time", rclcpp::ParameterValue(2.0));

  nav2_util::declare_parameter_if_not_declared(
      node, "graph_filepath", rclcpp::ParameterValue(
          ament_index_cpp::get_package_share_directory("nav2_route") +
          "/graphs/geojson/aws_graph.geojson"));

  route_frame_ = node->get_parameter("route_frame").as_string();
  base_frame_ = node->get_parameter("base_frame").as_string();
  max_planning_time_ = node->get_parameter("max_planning_time").as_double();
  graph_filepath_ = node->get_parameter("graph_filepath").as_string();

  RCLCPP_ERROR_STREAM(node->get_logger(), "Graph file path" << graph_filepath_);

  // Load graph and convert poses to the route frame, if required
  try {
    graph_loader_ = std::make_shared<GraphFileLoader>(node, tf_, route_frame_);
    if (!graph_loader_->loadGraphFromFile(graph_, id_to_graph_map_, graph_filepath_)) {
      return nav2_util::CallbackReturn::FAILURE;
    }

    // Precompute the graph's kd-tree
    node_spatial_tree_ = std::make_shared<NodeSpatialTree>();
    node_spatial_tree_->computeTree(graph_);

    // Create main planning algorithm
    route_planner_ = std::make_shared<RoutePlanner>();
    route_planner_->configure(node);

    // Create route tracking system
    route_tracker_ = std::make_shared<RouteTracker>();
    route_tracker_->configure(
      node, tf_, compute_and_track_route_server_, route_frame_, base_frame_);

    // Create Route to path conversion utility
    path_converter_ = std::make_shared<PathConverter>();
    path_converter_->configure(node);
  } catch (std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to configure route server: %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RouteServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  compute_route_server_->activate();
  compute_and_track_route_server_->activate();
  graph_vis_publisher_->on_activate();
  graph_vis_publisher_->publish(utils::toMsg(graph_, route_frame_, this->now()));

  // Add callback for dynamic parameters
  dyn_params_handler_ = this->add_on_set_parameters_callback(
    std::bind(&RouteServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RouteServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  compute_route_server_->deactivate();
  compute_and_track_route_server_->deactivate();
  graph_vis_publisher_->on_deactivate();
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RouteServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  compute_route_server_.reset();
  compute_and_track_route_server_.reset();
  graph_vis_publisher_.reset();
  set_graph_service_.reset();
  transform_listener_.reset();
  tf_.reset();
  graph_.clear();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RouteServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename GoalT>
NodeExtents
RouteServer::findStartandGoalNodeLocations(const std::shared_ptr<const GoalT> goal)
{
  // If not using the poses, then use the requests Node IDs to establish start and goal
  if (!goal->use_poses) {
    return {id_to_graph_map_.at(goal->start_id), id_to_graph_map_.at(goal->goal_id)};
  }

  // Find request start pose
  geometry_msgs::msg::PoseStamped start_pose, goal_pose = goal->goal;
  if (goal->use_start) {
    start_pose = goal->start;
  } else {
    if (!nav2_util::getCurrentPose(start_pose, *tf_, route_frame_, base_frame_)) {
      throw nav2_core::RouteTFError("Failed to obtain starting pose in: " + route_frame_);
    }
  }

  // If start or goal not provided in route_frame, transform
  if (start_pose.header.frame_id != route_frame_) {
    RCLCPP_INFO(
      get_logger(),
      "Request start pose not in %s frame. Converting %s to route server frame.",
      start_pose.header.frame_id.c_str(), route_frame_.c_str());
    if (!nav2_util::transformPoseInTargetFrame(start_pose, start_pose, *tf_, route_frame_)) {
      throw nav2_core::RouteTFError("Failed to transform starting pose to: " + route_frame_);
    }
  }

  if (goal_pose.header.frame_id != route_frame_) {
    RCLCPP_INFO(
      get_logger(),
      "Request goal pose not in %s frame. Converting %s to route server frame.",
      start_pose.header.frame_id.c_str(), route_frame_.c_str());
    if (!nav2_util::transformPoseInTargetFrame(goal_pose, goal_pose, *tf_, route_frame_) ) {
      throw nav2_core::RouteTFError("Failed to transform goal pose to: " + route_frame_);
    }
  }

  // Find closest route graph nodes to start and goal to plan between.
  // Note that these are the location indices in the graph, NOT the node IDs for easier starting
  // lookups for search. The route planner will convert them to node ids for route reporting.
  unsigned int start_route = 0, end_route = 0;
  if (!node_spatial_tree_->findNearestGraphNodeToPose(start_pose, start_route) ||
    !node_spatial_tree_->findNearestGraphNodeToPose(goal_pose, end_route))
  {
    throw nav2_core::IndeterminantNodesOnGraph(
            "Could not determine node closest to start or goal pose requested!");
  }

  return {start_route, end_route};
}

template<typename GoalT>
Route RouteServer::findRoute(const std::shared_ptr<const GoalT> goal)
{
  // Find the search boundaries
  auto [start_route, end_route] = findStartandGoalNodeLocations(goal);

  Route route;
  if (start_route == end_route) {
    // Succeed with a single-point route
    route.route_cost = 0.0;
    route.start_node = &graph_.at(start_route);
  } else {
    // Compute the route via graph-search, returns a node-edge sequence
    route = route_planner_->findRoute(graph_, start_route, end_route);
  }

  return route;
}

rclcpp::Duration
RouteServer::findPlanningDuration(const rclcpp::Time & start_time)
{
  auto cycle_duration = this->now() - start_time;
  if (max_planning_time_ && cycle_duration.seconds() > max_planning_time_) {
    RCLCPP_WARN(
      get_logger(),
      "Route planner missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
      1 / max_planning_time_, 1 / cycle_duration.seconds());
  }

  return cycle_duration;
}

template<typename T>
bool
RouteServer::isRequestValid(
  std::shared_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (!action_server || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return false;
  }

  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling route planning action.");
    action_server->terminate_all();
    return false;
  }

  return true;
}

void
RouteServer::computeRoute()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  auto goal = compute_route_server_->get_current_goal();
  auto result = std::make_shared<ComputeRouteResult>();
  RCLCPP_INFO(get_logger(), "Computing route to goal.");

  if (!isRequestValid(compute_route_server_)) {
    return;
  }

  if (compute_route_server_->is_preempt_requested()) {
    goal = compute_route_server_->accept_pending_goal();
  }

  try {
    // Find the route
    auto start_time = this->now();
    Route route = findRoute(goal);

    // Create a dense path for use and debugging visualization
    result->route = utils::toMsg(route, route_frame_, this->now());
    result->path = path_converter_->densify(route, route_frame_, this->now());
    result->planning_time = findPlanningDuration(start_time);
    compute_route_server_->succeeded_current(result);
  } catch (nav2_core::NoValidRouteCouldBeFound & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeRouteGoal::NO_VALID_ROUTE;
    compute_route_server_->terminate_current(result);
  } catch (nav2_core::TimedOut & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeRouteGoal::TIMEOUT;
    compute_route_server_->terminate_current(result);
  } catch (nav2_core::RouteTFError & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeRouteGoal::TF_ERROR;
    compute_route_server_->terminate_current(result);
  } catch (nav2_core::NoValidGraph & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeRouteGoal::NO_VALID_GRAPH;
    compute_route_server_->terminate_current(result);
  } catch (nav2_core::IndeterminantNodesOnGraph & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeRouteGoal::INDETERMINANT_NODES_ON_GRAPH;
    compute_route_server_->terminate_current(result);
  } catch (nav2_core::RouteException & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeRouteGoal::UNKNOWN;
    compute_route_server_->terminate_current(result);
  } catch (std::exception & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeRouteGoal::UNKNOWN;
    compute_route_server_->terminate_current(result);
  }
}

void
RouteServer::computeAndTrackRoute()
{
  auto goal = compute_and_track_route_server_->get_current_goal();
  auto result = std::make_shared<ComputeAndTrackRouteResult>();
  RCLCPP_INFO(get_logger(), "Computing and tracking route to goal.");

  try {
    while (rclcpp::ok()) {
      if (!isRequestValid(compute_and_track_route_server_)) {
        return;
      }

      if (compute_and_track_route_server_->is_preempt_requested()) {
        RCLCPP_INFO(get_logger(), "Computing new and tracking preempted route to goal.");
        goal = compute_and_track_route_server_->accept_pending_goal();
      }

      std::lock_guard<std::mutex> lock(dynamic_params_lock_);

      // Find the route
      auto start_time = this->now();
      // TODO(sm) prune passed objs in rerouting, provide optional start nodeid to override action?
      Route route = findRoute(goal);
      auto path = path_converter_->densify(route, route_frame_, this->now());
      findPlanningDuration(start_time);

      // blocks until re-route requested or task completion, publishes feedback
      switch (route_tracker_->trackRoute(route, path)) {
        case TrackerResult::COMPLETED:
          compute_and_track_route_server_->succeeded_current(result);
          return;
        case TrackerResult::REROUTE:
          break;
        case TrackerResult::INTERRUPTED:
          return;
      }
    }
  } catch (nav2_core::OperationFailed & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeAndTrackRouteGoal::OPERATION_FAILED;
    compute_and_track_route_server_->terminate_current(result);
  } catch (nav2_core::NoValidRouteCouldBeFound & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeAndTrackRouteGoal::NO_VALID_ROUTE;
    compute_and_track_route_server_->terminate_current(result);
  } catch (nav2_core::TimedOut & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeAndTrackRouteGoal::TIMEOUT;
    compute_and_track_route_server_->terminate_current(result);
  } catch (nav2_core::RouteTFError & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeAndTrackRouteGoal::TF_ERROR;
    compute_and_track_route_server_->terminate_current(result);
  } catch (nav2_core::NoValidGraph & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeAndTrackRouteGoal::NO_VALID_GRAPH;
    compute_and_track_route_server_->terminate_current(result);
  } catch (nav2_core::IndeterminantNodesOnGraph & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeAndTrackRouteGoal::INDETERMINANT_NODES_ON_GRAPH;
    compute_and_track_route_server_->terminate_current(result);
  } catch (nav2_core::RouteException & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeAndTrackRouteGoal::UNKNOWN;
    compute_and_track_route_server_->terminate_current(result);
  } catch (std::exception & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ComputeAndTrackRouteGoal::UNKNOWN;
    compute_and_track_route_server_->terminate_current(result);
  }
}

void RouteServer::setRouteGraph(
  const std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Request> request,
  std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Response> response)
{
  RCLCPP_INFO(get_logger(), "Setting new route graph: %s.", request->graph_filepath.c_str());

  if (!graph_loader_->loadGraphFromFile(graph_, id_to_graph_map_, request->graph_filepath)) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to set new route graph: %s!", request->graph_filepath.c_str());
    response->success = false;
    return;
  }

  // Re-compute the graph's kd-tree and publish new graph
  node_spatial_tree_->computeTree(graph_);
  graph_vis_publisher_->publish(utils::toMsg(graph_, route_frame_, this->now()));
  response->success = true;
}

template<typename GoalT>
void RouteServer::exceptionWarning(
  const std::shared_ptr<const GoalT> goal,
  const std::exception & ex)
{
  RCLCPP_WARN(
    get_logger(),
    "Route server failed on request: Start: [(%0.2f, %0.2f) / %i] Goal: [(%0.2f, %0.2f) / %i]:"
    " \"%s\"", goal->start.pose.position.x, goal->start.pose.position.y, goal->start_id,
    goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal_id, ex.what());
}

rcl_interfaces::msg::SetParametersResult
RouteServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  using rcl_interfaces::msg::ParameterType;
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "max_planning_time") {
        max_planning_time_ = parameter.as_double();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_route

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_route::RouteServer)
