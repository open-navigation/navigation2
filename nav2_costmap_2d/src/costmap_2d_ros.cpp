/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <nav2_costmap_2d/layered_costmap.h>
#include <nav2_costmap_2d/costmap_2d_ros.h>
#include <cstdio>
#include <string>
#include <sys/time.h>
#include <algorithm>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nav2_util/duration_conversions.h"

using namespace std;

namespace nav2_costmap_2d
{

void move_parameter(rclcpp::Node::SharedPtr old_h, rclcpp::Node::SharedPtr new_h, std::string name,
    bool should_delete = true)
{
  auto parameters_client_old = std::make_shared<rclcpp::SyncParametersClient>(old_h);
  auto parameters_client_new = std::make_shared<rclcpp::SyncParametersClient>(new_h);

  if (!parameters_client_old->has_parameter(name)) {
    return;
  }

  std::vector<rclcpp::Parameter> param;
  param = parameters_client_old->get_parameters({name});
  parameters_client_new->set_parameters(param);
  //TODO(bpwilcox): port delete parameter from client?
  //if (should_delete) old_h.deleteParam(name);
}

Costmap2DROS::Costmap2DROS(const std::string & name, tf2_ros::Buffer & tf)
  : layered_costmap_(NULL),
  name_(name),
  tf_(tf),
  transform_tolerance_(0.3),
  map_update_thread_shutdown_(false),
  stop_updates_(false),
  initialized_(true),
  stopped_(false),
  robot_stopped_(false),
  map_update_thread_(NULL),
  last_publish_(0),
  plugin_loader_("nav2_costmap_2d", "nav2_costmap_2d::Layer"),
  publisher_(NULL),
  publish_cycle_(1),
  footprint_padding_(0.0)
{
  tf2::toMsg(tf2::Transform::getIdentity(), old_pose_.pose);

  private_nh_ = rclcpp::Node::make_shared(name_);

  // get two frames
  auto private_parameters_client = std::make_shared<rclcpp::SyncParametersClient>(private_nh_);
  global_frame_ =
      private_parameters_client->get_parameter<std::string>("global_frame", std::string("map"));
  robot_base_frame_ =
      private_parameters_client->get_parameter<std::string>("robot_base_frame", std::string(
        "base_link"));

  rclcpp::Time last_error = private_nh_->now();
  std::string tf_error;

  // we need to make sure that the transform between the robot base frame and the global frame is available
  while (rclcpp::ok() &&
      !tf_.canTransform(global_frame_, robot_base_frame_, tf2_ros::fromMsg(rclcpp::Time()),
        tf2::durationFromSec(0.1), &tf_error))
  {
    rclcpp::spin_some(private_nh_);
    if (last_error + nav2_util::durationFromSeconds(5.0) < private_nh_->now()) {
      RCLCPP_WARN(rclcpp::get_logger(
            "nav2_costmap_2d"),
          "Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
          robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
      last_error = private_nh_->now();
    }
    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation.
    tf_error.clear();
  }

  // check if we want a rolling window version of the costmap
  bool rolling_window, track_unknown_space, always_send_full_costmap;

  rolling_window = private_parameters_client->get_parameter<bool>("rolling_window", false);
  track_unknown_space = private_parameters_client->get_parameter<bool>("track_unknown_space", false);
  always_send_full_costmap = private_parameters_client->get_parameter<bool>(
      "always_send_full_costmap", false);

  layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window, track_unknown_space);

  if (!private_parameters_client->has_parameter("plugins")) {
    resetOldParameters(private_nh_);
  }

  if (private_parameters_client->has_parameter("plugins")) {
    XmlRpc::XmlRpcValue my_list;
    my_list = private_parameters_client->get_parameter<XmlRpc::XmlRpcValue>("plugins");

    for (int32_t i = 0; i < my_list.size(); ++i) {
      std::string pname = static_cast<std::string>(my_list[i]["name"]);
      std::string type = static_cast<std::string>(my_list[i]["type"]);
      RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "Using plugin \"%s\"", pname.c_str());

      std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(type);
      layered_costmap_->addPlugin(plugin);
      plugin->initialize(layered_costmap_, name + "/" + pname, &tf_);
    }
  }

  // subscribe to the footprint topic
  std::string topic_param, topic;
  if (!private_parameters_client->has_parameter("footprint_topic")) {
    topic_param = "footprint_topic";
  }
  topic =
      private_parameters_client->get_parameter<std::string>(topic_param, std::string("footprint"));
  footprint_sub_ = private_nh_->create_subscription<geometry_msgs::msg::Polygon>(topic,
      std::bind(&Costmap2DROS::setUnpaddedRobotFootprintPolygon, this, std::placeholders::_1));

  if (!private_parameters_client->has_parameter("published_footprint_topic")) {
    topic_param = "published_footprint";
  }
  topic =
      private_parameters_client->get_parameter<std::string>(topic_param,
      std::string("oriented_footprint"));
  footprint_pub_ = private_nh_->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "footprint", rmw_qos_profile_default);

  setUnpaddedRobotFootprint(makeFootprintFromParams(private_nh_));

  publisher_ = new Costmap2DPublisher(private_nh_,
      layered_costmap_->getCostmap(), global_frame_, "costmap",
      always_send_full_costmap);

  // create a thread to handle updating the map
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  // Create a timer to check if the robot is moving
  robot_stopped_ = false;
  timer_ = private_nh_->create_wall_timer(100ms, std::bind(&Costmap2DROS::movementCB, this));

  // TODO(bpwilcox): resolve dynamic reconfigure dependencies
  //dsrv_ = new dynamic_reconfigure::Server<Costmap2DConfig>(ros::NodeHandle("~/" + name));
  //dynamic_reconfigure::Server<Costmap2DConfig>::CallbackType cb = std::bind(&Costmap2DROS::reconfigureCB, this, _1,
  //                                                                            _2);
  //dsrv_->setCallback(cb);
}

void Costmap2DROS::setUnpaddedRobotFootprintPolygon(
    const geometry_msgs::msg::Polygon::SharedPtr footprint)
{
  setUnpaddedRobotFootprint(toPointVector(footprint));
}

Costmap2DROS::~Costmap2DROS()
{
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL) {
    map_update_thread_->join();
    delete map_update_thread_;
  }
  if (publisher_ != NULL) {
    delete publisher_;
  }

  delete layered_costmap_;
  // TODO(bpwilcox): resolve dynamic reconfigure dependencies
  //delete dsrv_;
}

void Costmap2DROS::resetOldParameters(rclcpp::Node::SharedPtr nh)
{
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "Loading from pre-hydro parameter style");
  bool flag;
  std::string s;
  std::vector<XmlRpc::XmlRpcValue> plugins;

  XmlRpc::XmlRpcValue::ValueStruct map;
  SuperValue super_map;
  SuperValue super_array;

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(nh);

  if (parameters_client->has_parameter("static_map")) {
    flag = parameters_client->get_parameter<bool>("static_map");
    if (flag) {
      map["name"] = XmlRpc::XmlRpcValue("static_layer");
      map["type"] = XmlRpc::XmlRpcValue("nav2_costmap_2d::StaticLayer");
      super_map.setStruct(&map);
      plugins.push_back(super_map);

      auto map_layer = rclcpp::Node::make_shared("static_layer", std::string(nh->get_name()));

      move_parameter(nh, map_layer, "map_topic");
      move_parameter(nh, map_layer, "unknown_cost_value");
      move_parameter(nh, map_layer, "lethal_cost_threshold");
      move_parameter(nh, map_layer, "track_unknown_space", false);
    }
  }
  auto obstacles = rclcpp::Node::make_shared("obstacle_layer", std::string(nh->get_name()));
  //ros::NodeHandle obstacles(nh, "obstacle_layer");

  if (parameters_client->has_parameter("map_type")) {
    s = parameters_client->get_parameter<std::string>("map_type");
    if (s == "voxel") {
      map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
      map["type"] = XmlRpc::XmlRpcValue("nav2_costmap_2d::VoxelLayer");
      super_map.setStruct(&map);
      plugins.push_back(super_map);

      move_parameter(nh, obstacles, "origin_z");
      move_parameter(nh, obstacles, "z_resolution");
      move_parameter(nh, obstacles, "z_voxels");
      move_parameter(nh, obstacles, "mark_threshold");
      move_parameter(nh, obstacles, "unknown_threshold");
      move_parameter(nh, obstacles, "publish_voxel_map");
    }
  } else {
    map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
    map["type"] = XmlRpc::XmlRpcValue("nav2_costmap_2d::ObstacleLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);
  }

  move_parameter(nh, obstacles, "max_obstacle_height");
  move_parameter(nh, obstacles, "raytrace_range");
  move_parameter(nh, obstacles, "obstacle_range");
  move_parameter(nh, obstacles, "track_unknown_space", true);

  s = parameters_client->get_parameter<std::string>("observation_sources", std::string(""));
  std::stringstream ss(s);
  std::string source;
  while (ss >> source) {
    move_parameter(nh, obstacles, source);
  }
  move_parameter(nh, obstacles, "observation_sources");

  auto inflation = rclcpp::Node::make_shared("obstacle_layer", std::string(nh->get_name()));
  //ros::NodeHandle inflation(nh, "inflation_layer");

  move_parameter(nh, inflation, "cost_scaling_factor");
  move_parameter(nh, inflation, "inflation_radius");
  map["name"] = XmlRpc::XmlRpcValue("inflation_layer");
  map["type"] = XmlRpc::XmlRpcValue("nav2_costmap_2d::InflationLayer");
  super_map.setStruct(&map);
  plugins.push_back(super_map);

  super_array.setArray(&plugins);
  // TODO(bpwilcox): parameter client doesn't support setting xmlrpc data structure
  //nh.setParam("plugins", super_array);
  //auto set_parameters_results = parameters_client->set_parameters({
  //  rclcpp::Parameter("plugins", super_array)
  //});

}
// TODO(bpwilcox): resolve dynamic reconfigure dependencies
/*
void Costmap2DROS::reconfigureCB(nav2_costmap_2d::Costmap2DConfig &config, uint32_t level)
{
  transform_tolerance_ = config.transform_tolerance;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency = config.update_frequency;

  double map_publish_frequency = config.publish_frequency;
  if (map_publish_frequency > 0)
    publish_cycle = ros::Duration(1 / map_publish_frequency);
  else
    publish_cycle = ros::Duration(-1);

  // find size parameters
  double map_width_meters = config.width, map_height_meters = config.height, resolution = config.resolution, origin_x =
             config.origin_x,
         origin_y = config.origin_y;

  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }

  // If the padding has changed, call setUnpaddedRobotFootprint() to
  // re-apply the padding.
  if (footprint_padding_ != config.footprint_padding)
  {
    footprint_padding_ = config.footprint_padding;
    setUnpaddedRobotFootprint(unpadded_footprint_);
  }

  readFootprintFromConfig(config, old_config_);

  old_config_ = config;

  map_update_thread_ = new std::thread(std::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
} */

// TODO(bpwilcox): resolve dynamic reconfigure dependencies
/*
void Costmap2DROS::readFootprintFromConfig(const nav2_costmap_2d::Costmap2DConfig &new_config,
                                           const nav2_costmap_2d::Costmap2DConfig &old_config)
{
  // Only change the footprint if footprint or robot_radius has
  // changed.  Otherwise we might overwrite a footprint sent on a
  // topic by a dynamic_reconfigure call which was setting some other
  // variable.
  if (new_config.footprint == old_config.footprint &&
      new_config.robot_radius == old_config.robot_radius)
  {
    return;
  }

  if (new_config.footprint != "" && new_config.footprint != "[]")
  {
    std::vector<geometry_msgs::msg::Point> new_footprint;
    if (makeFootprintFromString(new_config.footprint, new_footprint))
    {
        setUnpaddedRobotFootprint(new_footprint);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"),"Invalid footprint string from dynamic reconfigure");
    }
  }
  else
  {
    // robot_radius may be 0, but that must be intended at this point.
    setUnpaddedRobotFootprint(makeFootprintFromRadius(new_config.robot_radius));
  }
} */


void Costmap2DROS::setUnpaddedRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  padFootprint(padded_footprint_, footprint_padding_);

  layered_costmap_->setFootprint(padded_footprint_);
}

// TODO(bpwilcox): resolve dynamic reconfigure dependencies

void Costmap2DROS::movementCB()
{
  // don't allow configuration to happen while this check occurs
  // std::recursive_mutex::scoped_lock mcl(configuration_mutex_);

  geometry_msgs::msg::PoseStamped new_pose;
  if (!getRobotPose(new_pose)) {
    RCLCPP_WARN(rclcpp::get_logger(
          "nav2_costmap_2d"), "Could not get robot pose, cancelling reconfiguration");
    robot_stopped_ = false;
  }
  // make sure that the robot is not moving
  else {
    old_pose_ = new_pose;

    robot_stopped_ = (tf2::Vector3(old_pose_.pose.position.x, old_pose_.pose.position.y,
          old_pose_.pose.position.z).distance(tf2::Vector3(new_pose.pose.position.x,
            new_pose.pose.position.y, new_pose.pose.position.z)) < 1e-3) &&
        (tf2::Quaternion(old_pose_.pose.orientation.x,
          old_pose_.pose.orientation.y,
          old_pose_.pose.orientation.z,
          old_pose_.pose.orientation.w).angle(tf2::Quaternion(new_pose.pose.orientation.x,
            new_pose.pose.orientation.y,
            new_pose.pose.orientation.z,
            new_pose.pose.orientation.w)) < 1e-3);
  }
}

void Costmap2DROS::mapUpdateLoop(double frequency)
{
  // the user might not want to run the loop every cycle
  if (frequency == 0.0) {
    return;
  }

  rclcpp::Rate r(frequency);
  while (rclcpp::ok() && !map_update_thread_shutdown_) {
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);

    updateMap();

    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "Map update time: %.9f", t_diff);
    if (publish_cycle_.nanoseconds() > 0 && layered_costmap_->isInitialized()) {
      unsigned int x0, y0, xn, yn;
      layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
      publisher_->updateBounds(x0, xn, y0, yn);

      rclcpp::Time now = private_nh_->now();
      if (last_publish_.nanoseconds() + publish_cycle_.nanoseconds() < now.nanoseconds()) {
        publisher_->publishCostmap();
        last_publish_ = now;
      }
    }
    r.sleep();
    // make sure to sleep for the remainder of our cycle time

    if (r.period() > tf2::durationFromSec(1 / frequency)) {
      RCLCPP_WARN(rclcpp::get_logger(
            "nav2_costmap_2d"),
          "Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds",
          frequency,
          r.period());
    }
  }
}

void Costmap2DROS::updateMap()
{
  if (!stop_updates_) {
    // get global pose
    geometry_msgs::msg::PoseStamped pose;
    if (getRobotPose(pose)) {
      double x = pose.pose.position.x,
        y = pose.pose.position.y,
        yaw = tf2::getYaw(pose.pose.orientation);

      layered_costmap_->updateMap(x, y, yaw);

      geometry_msgs::msg::PolygonStamped footprint;
      footprint.header.frame_id = global_frame_;
      footprint.header.stamp = private_nh_->now();
      transformFootprint(x, y, yaw, padded_footprint_, footprint);
      footprint_pub_->publish(footprint);

      initialized_ = true;
    }
  }
}

void Costmap2DROS::start()
{
  std::vector<std::shared_ptr<Layer> > * plugins = layered_costmap_->getPlugins();
  // check if we're stopped or just paused
  if (stopped_) {
    // if we're stopped we need to re-subscribe to topics
    for (vector<std::shared_ptr<Layer> >::iterator plugin = plugins->begin();
        plugin != plugins->end();
        ++plugin)
    {
      (*plugin)->activate();
    }
    stopped_ = false;
  }
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(100.0);
  while (rclcpp::ok() && !initialized_) {
    r.sleep();
  }
}

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  std::vector<std::shared_ptr<Layer> > * plugins = layered_costmap_->getPlugins();
  // unsubscribe from topics
  for (vector<std::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(100.0);
  while (!initialized_) {
    r.sleep();
  }
}


void Costmap2DROS::resetLayers()
{
  Costmap2D * top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  std::vector<std::shared_ptr<Layer> > * plugins = layered_costmap_->getPlugins();
  for (vector<std::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
}

bool Costmap2DROS::getRobotPose(geometry_msgs::msg::PoseStamped & global_pose) const
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::msg::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);

  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = rclcpp::Time();

  rclcpp::Time current_time = private_nh_->now();  // save time for checking tf delay later
  // get the global pose of the robot
  try {
    tf_.transform(robot_pose, global_pose, global_frame_);
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger(
          "nav2_costmap_2d"), "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger(
          "nav2_costmap_2d"), "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger(
          "nav2_costmap_2d"), "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout

  //TODO(bpwilcox): use toSec() function in more recent rclcpp branch
  if (tf2::timeToSec(tf2_ros::fromMsg(current_time)) -
      tf2::timeToSec(tf2_ros::fromMsg(global_pose.header.stamp)) > transform_tolerance_)
  {
    RCLCPP_WARN(rclcpp::get_logger(
          "nav2_costmap_2d"),
        "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
        tf2::timeToSec(tf2_ros::fromMsg(current_time)),
        tf2::timeToSec(tf2_ros::fromMsg(global_pose.header.stamp)), transform_tolerance_);

    return false;
  }

  return true;
}

void Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::msg::Point> & oriented_footprint)
const
{
  geometry_msgs::msg::PoseStamped global_pose;
  if (!getRobotPose(global_pose)) {
    return;
  }

  double yaw = tf2::getYaw(global_pose.pose.orientation);
  transformFootprint(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
      padded_footprint_, oriented_footprint);
}

}  // namespace nav2_costmap_2d
