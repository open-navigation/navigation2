// Copyright (c) 2021 RoboTech Vision
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

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"

#include "nav2_behavior_tree/plugins/action/truncate_path_local_action.hpp"

namespace nav2_behavior_tree
{

TruncatePathLocal::TruncatePathLocal(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");
}

inline BT::NodeStatus TruncatePathLocal::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  double distance_forward, distance_backward;
  geometry_msgs::msg::PoseStamped pose;
  double angular_distance_weight;
  double max_robot_pose_search_dist;

  getInput("distance_forward", distance_forward);
  getInput("distance_backward", distance_backward);
  getInput("angular_distance_weight", angular_distance_weight);
  getInput("max_robot_pose_search_dist", max_robot_pose_search_dist);

  bool path_pruning = std::isfinite(max_robot_pose_search_dist);
  nav_msgs::msg::Path new_path;
  getInput("input_path", new_path);
  if (!path_pruning || new_path != path_) {
    path_ = new_path;
    closest_pose_detection_begin_ = path_.poses.begin();
  }

  if (!getRobotPose(path_.header.frame_id, pose)) {
    return BT::NodeStatus::FAILURE;
  }

  if (path_.poses.empty()) {
    setOutput("output_path", path_);
    return BT::NodeStatus::SUCCESS;
  }

  auto closest_pose_detection_end = path_.poses.end();
  if (path_pruning) {
    closest_pose_detection_end = nav2_util::geometry_utils::first_after_integrated_distance(
      closest_pose_detection_begin_, path_.poses.end(), max_robot_pose_search_dist);
  }

  // find the closest pose on the path
  auto current_pose = nav2_util::geometry_utils::min_by(
    closest_pose_detection_begin_, closest_pose_detection_end,
    [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
      return poseDistance(pose, ps, angular_distance_weight);
    });

  if (path_pruning) {
    closest_pose_detection_begin_ = current_pose;
  }

  // expand forwards to extract desired length
  auto forward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
    current_pose, path_.poses.end(), distance_forward);

  // expand backwards to extract desired length
  // Note: current_pose + 1 is used because reverse iterator points to a cell before it
  auto backward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
    std::reverse_iterator(current_pose + 1), path_.poses.rend(), distance_backward);

  nav_msgs::msg::Path output_path;
  output_path.header = path_.header;
  output_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(
    backward_pose_it.base(), forward_pose_it);
  setOutput("output_path", output_path);

  return BT::NodeStatus::SUCCESS;
}

inline bool TruncatePathLocal::getRobotPose(
  std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose)
{
  if (!getInput("pose", pose)) {
    std::string robot_frame;
    if (!getInput("robot_frame", robot_frame)) {
      RCLCPP_ERROR(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Neither pose nor robot_frame specified for %s", name().c_str());
      return false;
    }
    double transform_tolerance;
    getInput("transform_tolerance", transform_tolerance);
    if (!nav2_util::getCurrentPose(
        pose, *tf_buffer_, path_frame_id, robot_frame, transform_tolerance))
    {
      RCLCPP_WARN(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Failed to lookup current robot pose for %s", name().c_str());
      return false;
    }
  }
  return true;
}

double
TruncatePathLocal::poseDistance(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2,
  const double angular_distance_weight)
{
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  // taking angular distance into account in addition to spatial distance
  // (to improve picking a correct pose near cusps and loops)
  tf2::Quaternion q1;
  tf2::convert(pose1.pose.orientation, q1);
  tf2::Quaternion q2;
  tf2::convert(pose2.pose.orientation, q2);
  double da = angular_distance_weight * std::abs(q1.angleShortestPath(q2));
  return std::sqrt(dx * dx + dy * dy + da * da);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::TruncatePathLocal>(
    "TruncatePathLocal");
}
