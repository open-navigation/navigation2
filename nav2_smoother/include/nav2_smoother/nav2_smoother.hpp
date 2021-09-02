// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_
#define NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_smoother
{

/**
 * @class nav2_smoother::SmootherServer
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed SmoothPath action server.
 */
class SmootherServer : public nav2_util::LifecycleNode
{
public:
  using SmootherMap = std::unordered_map<std::string, nav2_core::Smoother::Ptr>;

  /**
   * @brief Constructor for nav2_smoother::SmootherServer
   */
  SmootherServer();
  /**
   * @brief Destructor for nav2_smoother::SmootherServer
   */
  ~SmootherServer();

protected:
  /**
   * @brief Configures smoother parameters and member variables
   *
   * Configures smoother plugin and costmap; Initialize odom subscriber,
   * velocity publisher and smooth path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize smoother
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates smoother, costmap, velocity publisher and smooth path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates smooth path action server, smoother, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Smoother and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = nav2_msgs::action::SmoothPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the SmoothPath action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief SmoothPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to smoother received from action client. Local
   * section of the path is optimized using smoother.
   * @throw nav2_core::PlannerException
   */
  void smoothPlan();

  /**
   * @brief Wait for costmap to be valid with updated sensor data or repopulate after a
   * clearing recovery. Blocks until true without timeout.
   */
  void waitForCostmap();

  /**
   * @brief Find the valid smoother ID name for the given request
   *
   * @param c_name The requested smoother name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid smoother to use
   */
  bool findSmootherId(const std::string & c_name, std::string & name);

  /**
   * @brief Finds local section of path around robot
   * @param goal Goal received from action server
   */
  void findLocalSection(const std::shared_ptr<const typename Action::Goal> &goal, std::size_t &begin, std::size_t &end);

  /**
   * @brief Distance between two poses. Uses angular_distance_weight param to include angular distance.
   * @param path Path received from action server
   */
  double poseDistance(const geometry_msgs::msg::PoseStamped &pose1, const geometry_msgs::msg::PoseStamped &pose2);

  // The smoother needs a costmap subscriber and uses tf to acquire current robot pose
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers and subscribers
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  
  // Smoother Plugins
  pluginlib::ClassLoader<nav2_core::Smoother> lp_loader_;
  SmootherMap smoothers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> smoother_ids_;
  std::vector<std::string> smoother_types_;
  std::string smoother_ids_concat_, current_smoother_;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  double optimization_length_;
  double optimization_length_backwards_;
  double transform_tolerance_;
  double angular_distance_weight_;
  std::string robot_frame_id_;

};

}  // namespace nav2_smoother

#endif  // NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_
