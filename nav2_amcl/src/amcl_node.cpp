/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include "nav2_amcl/amcl_node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "message_filters/subscriber.h"
#include "nav2_util/angleutils.hpp"
#include "nav2_util/duration_conversions.hpp"
#include "nav2_util/map_service_client.hpp"
#include "nav2_util/pf/pf.hpp"
#include "nav2_util/string_utils.hpp"
#include "nav2_util/sensors/laser/laser.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace nav2_amcl
{

AmclNode::AmclNode()
: nav2_util::LifecycleNode("amcl", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("alpha1", rclcpp::ParameterValue(0.2));
  declare_parameter("alpha2", rclcpp::ParameterValue(0.2));
  declare_parameter("alpha3", rclcpp::ParameterValue(0.2));
  declare_parameter("alpha4", rclcpp::ParameterValue(0.2));
  declare_parameter("alpha5", rclcpp::ParameterValue(0.2));
  declare_parameter("base_frame_id", rclcpp::ParameterValue(std::string("base_footprint")));
  declare_parameter("beam_skip_distance", rclcpp::ParameterValue(0.5));
  declare_parameter("beam_skip_error_threshold", rclcpp::ParameterValue(0.9));
  declare_parameter("beam_skip_threshold", rclcpp::ParameterValue(0.3));
  declare_parameter("do_beamskip", rclcpp::ParameterValue(false));
  declare_parameter("global_frame_id", rclcpp::ParameterValue(std::string("map")));
  declare_parameter("lambda_short", rclcpp::ParameterValue(0.1));
  declare_parameter("laser_likelihood_max_dist", rclcpp::ParameterValue(2.0));
  declare_parameter("laser_max_range", rclcpp::ParameterValue(100.0));
  declare_parameter("laser_min_range", rclcpp::ParameterValue(-1.0));
  declare_parameter("laser_model_type", rclcpp::ParameterValue(std::string("likelihood_field")));
  declare_parameter("max_beams", rclcpp::ParameterValue(60));
  declare_parameter("max_particles", rclcpp::ParameterValue(2000));
  declare_parameter("min_particles", rclcpp::ParameterValue(500));
  declare_parameter("odom_frame_id", rclcpp::ParameterValue(std::string("odom")));
  declare_parameter("pf_err", rclcpp::ParameterValue(0.05));
  declare_parameter("pf_z", rclcpp::ParameterValue(0.99));
  declare_parameter("recovery_alpha_fast", rclcpp::ParameterValue(0.0));
  declare_parameter("recovery_alpha_slow", rclcpp::ParameterValue(0.0));
  declare_parameter("resample_interval", rclcpp::ParameterValue(1));
  declare_parameter("robot_model_type", rclcpp::ParameterValue(std::string("differential")));
  declare_parameter("save_pose_rate", rclcpp::ParameterValue(0.5));
  declare_parameter("sigma_hit", rclcpp::ParameterValue(0.2));
  declare_parameter("tf_broadcast", rclcpp::ParameterValue(true));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(1.0));
  declare_parameter("update_min_a", rclcpp::ParameterValue(0.2));
  declare_parameter("update_min_d", rclcpp::ParameterValue(0.25));
  declare_parameter("z_hit", rclcpp::ParameterValue(0.5));
  declare_parameter("z_max", rclcpp::ParameterValue(0.05));
  declare_parameter("z_rand", rclcpp::ParameterValue(0.5));
  declare_parameter("z_short", rclcpp::ParameterValue(0.05));
}

AmclNode::~AmclNode()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
AmclNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initParameters();
  initMap();
  initTransforms();
  initMessageFilters();
  initPubSub();
  initServices();
  initOdometry();
  initParticleFilter();
  initLaserScan();

  return nav2_util::CallbackReturn::SUCCESS;
}

void
AmclNode::waitForTransforms()
{
  std::string tf_error;

  RCLCPP_INFO(get_logger(), "Checking that transform thread is ready");

  while (rclcpp::ok() &&
    !tf_buffer_->canTransform(global_frame_id_, odom_frame_id_, tf2::TimePointZero,
    tf2::durationFromSec(1.0), &tf_error))
  {
    RCLCPP_INFO(get_logger(), "Timed out waiting for transform from %s to %s"
      " to become available, tf error: %s",
      odom_frame_id_.c_str(), global_frame_id_.c_str(), tf_error.c_str());

    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation.
    tf_error.clear();
  }
}

nav2_util::CallbackReturn
AmclNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Lifecycle publishers must be explicitly activated
  pose_pub_->on_activate();
  particlecloud_pub_->on_activate();

  first_pose_sent_ = false;
  amcl_node_ready_ = false;

  // Keep track of whether we're in the active state. We won't
  // process incoming callbacks until we are
  active_ = true;

  if (init_pose_received_on_inactive) {
    handleInitialPose(
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(last_published_pose_));
  }

  // Make sure amcl is ready before continuing
  while (!amcl_node_ready_) {
    std::this_thread::sleep_for(100ms);
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AmclNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  active_ = false;

  // Lifecycle publishers must be explicitly deactivated
  pose_pub_->on_deactivate();
  particlecloud_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AmclNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Get rid of the inputs first (services and message filter input), so we
  // don't continue to process incoming messages
  global_loc_srv_.reset();
  nomotion_update_srv_.reset();
  initial_pose_sub_.reset();
  laser_scan_connection_.disconnect();
  laser_scan_filter_.reset();
  laser_scan_sub_.reset();

  // Map
  map_free(map_);
  map_ = nullptr;
  free_space_indices.resize(0);

  // Transforms
  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  // PubSub
  pose_pub_.reset();
  particlecloud_pub_.reset();

  // Odometry
  motion_model_.reset();

  // Particle Filter
  pf_free(pf_);
  pf_ = nullptr;

  // Laser Scan
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AmclNode::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AmclNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
AmclNode::checkLaserReceived()
{
  if (last_laser_received_ts_.nanoseconds() == 0) {
    RCLCPP_WARN(
      get_logger(), "Laser scan has not been received"
      " (and thus no pose updates have been published)."
      " Verify that data is being published on the %s topic.", scan_topic_);
    return;
  }

  rclcpp::Duration d = now() - last_laser_received_ts_;
  if (d.nanoseconds() * 1e-9 > laser_check_interval_.count()) {
    RCLCPP_WARN(
      get_logger(), "No laser scan received (and thus no pose updates have been published) for %f"
      " seconds.  Verify that data is being published on the %s topic.",
      d.nanoseconds() * 1e-9, scan_topic_);
  }
}

bool
AmclNode::checkElapsedTime(std::chrono::seconds check_interval, rclcpp::Time last_time)
{
  rclcpp::Duration elapsed_time = now() - last_time;
  if (elapsed_time.nanoseconds() * 1e-9 > check_interval.count()) {
    return true;
  }
  return false;
}

#if NEW_UNIFORM_SAMPLING
std::vector<std::pair<int, int>> AmclNode::free_space_indices;
#endif

bool
AmclNode::getOdomPose(
  geometry_msgs::msg::PoseStamped & odom_pose,
  double & x, double & y, double & yaw,
  const rclcpp::Time & sensor_timestamp, const std::string & frame_id)
{
  // Get the robot's pose
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = nav2_util::strip_leading_slash(frame_id);
  ident.header.stamp = sensor_timestamp;
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

  try {
    tf_buffer_->transform(ident, odom_pose, odom_frame_id_);
  } catch (tf2::TransformException e) {
    ++scan_error_count_;
    if (scan_error_count_ % 20 == 0) {
      RCLCPP_ERROR(
        get_logger(), "(%d) consecutive laser scan transforms failed: (%s)", scan_error_count_,
        e.what());
    }
    return false;
  }

  scan_error_count_ = 0;  // reset since we got a good transform
  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);

  return true;
}

pf_vector_t
AmclNode::uniformPoseGenerator(void * arg)
{
  map_t * map = reinterpret_cast<map_t *>(arg);

#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int, int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale) / 2.0 - map->origin_x;
  max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
  min_y = (map->size_y * map->scale) / 2.0 - map->origin_y;
  max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;

  pf_vector_t p;

  RCLCPP_DEBUG(get_logger(), "Generating new uniform sample");
  for (;; ) {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i, j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if (MAP_VALID(map, i, j) && (map->cells[MAP_INDEX(map, i, j)].occ_state == -1)) {
      break;
    }
  }
#endif
  return p;
}

void
AmclNode::globalLocalizationCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  RCLCPP_INFO(get_logger(), "Initializing with uniform distribution");

  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
    reinterpret_cast<void *>(map_));
  RCLCPP_INFO(get_logger(), "Global initialisation done!");
  initial_pose_is_known_ = true;
  pf_init_ = false;
}

// force nomotion updates (amcl updating without requiring motion)
void
AmclNode::nomotionUpdateCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  RCLCPP_INFO(get_logger(), "Requesting no-motion update");
  force_update_ = true;
}

void
AmclNode::initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");

  if (msg->header.frame_id == "") {
    // This should be removed at some point
    RCLCPP_WARN(get_logger(),
      "Received initial pose with empty frame_id. You should always supply a frame_id.");
    return;
  }
  if (nav2_util::strip_leading_slash(msg->header.frame_id) != global_frame_id_) {
    RCLCPP_WARN(get_logger(),
      "Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
      nav2_util::strip_leading_slash(msg->header.frame_id).c_str(),
      global_frame_id_.c_str());
    return;
  }
  // Overriding last published pose to initial pose
  last_published_pose_ = *msg;

  if (!active_) {
    init_pose_received_on_inactive = true;
    RCLCPP_WARN(get_logger(), "Received initial pose request, "
      "but AMCL is not yet in the active state");
    return;
  }
  handleInitialPose(msg);
}

void
AmclNode::handleInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  initParticleFilter();

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  geometry_msgs::msg::TransformStamped tx_odom;
  try {
    rclcpp::Time rclcpp_time = now();
    tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

    // Check if the transform is available
    tx_odom = tf_buffer_->lookupTransform(base_frame_id_, tf2_ros::fromMsg(msg->header.stamp),
        base_frame_id_, tf2_time, odom_frame_id_);
  } catch (tf2::TransformException e) {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if (sent_first_transform_) {
      RCLCPP_WARN(get_logger(), "Failed to transform initial pose in time (%s)", e.what());
    }
    tf2::impl::Converter<false, true>::convert(tf2::Transform::getIdentity(), tx_odom.transform);
  }

  tf2::Transform tx_odom_tf2;
  tf2::impl::Converter<true, false>::convert(tx_odom.transform, tx_odom_tf2);

  tf2::Transform pose_old;
  tf2::impl::Converter<true, false>::convert(msg->pose.pose, pose_old);

  tf2::Transform pose_new = pose_old * tx_odom_tf2;

  // Transform into the global frame

  RCLCPP_INFO(get_logger(), "Setting pose (%.6f): %.3f %.3f %.3f",
    now().nanoseconds() * 1e-9,
    pose_new.getOrigin().x(),
    pose_new.getOrigin().y(),
    tf2::getYaw(pose_new.getRotation()));

  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = tf2::getYaw(pose_new.getRotation());

  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      pf_init_pose_cov.m[i][j] = msg->pose.covariance[6 * i + j];
    }
  }

  pf_init_pose_cov.m[2][2] = msg->pose.covariance[6 * 5 + 5];

  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;
  init_pose_received_on_inactive = false;
  initial_pose_is_known_ = true;
}

void
AmclNode::laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
  // Since the sensor data is continually being published by the simulator or robot,
  // we don't want our callbacks to fire until we're in the active state
  if (!active_) {return;}

  std::string laser_scan_frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  last_laser_received_ts_ = now();
  int laser_index = -1;
  geometry_msgs::msg::PoseStamped laser_pose;

  // Do we have the base->base_laser Tx yet?
  if (frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end()) {
    if (!addNewScanner(laser_index, laser_scan, laser_scan_frame_id, laser_pose)) {
      return;  // could not find transform
    }
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if (!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
    laser_scan->header.stamp, base_frame_id_))
  {
    RCLCPP_ERROR(get_logger(), "Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();
  bool force_publication = false;
  if (!pf_init_) {
    // Pose at last filter update
    pf_odom_pose_ = pose;
    pf_init_ = true;

    for (unsigned int i = 0; i < lasers_update_.size(); i++) {
      lasers_update_[i] = true;
    }

    force_publication = true;
    resample_count_ = 0;
  } else {
    // Set the laser update flags
    if (shouldUpdateFilter(pose, delta)) {
      for (unsigned int i = 0; i < lasers_update_.size(); i++) {
        lasers_update_[i] = true;
      }
    }
    if (lasers_update_[laser_index]) {
      motion_model_->odometryUpdate(pf_, pose, delta);
    }
    force_update_ = false;
  }

  bool resampled = false;

  // If the robot has moved, update the filter
  if (lasers_update_[laser_index]) {
    updateFilter(laser_index, laser_scan, pose);

    // Resample the particles
    if (!(++resample_count_ % resample_interval_)) {
      pf_update_resample(pf_);
      resampled = true;
    }

    pf_sample_set_t * set = pf_->sets + pf_->current_set;
    RCLCPP_DEBUG(get_logger(), "Num samples: %d\n", set->sample_count);

    if (!force_update_) {
      publishParticleCloud(set);
    }
  }
  if (resampled || force_publication || !first_pose_sent_) {
    amcl_hyp_t max_weight_hyps;
    std::vector<amcl_hyp_t> hyps;
    int max_weight_hyp = -1;
    if (getMaxWeightHyp(hyps, max_weight_hyps, max_weight_hyp)) {
      publishAmclPose(laser_scan, hyps, max_weight_hyp);
      calculateMaptoOdomTransform(laser_scan, hyps, max_weight_hyp);

      if (tf_broadcast_ == true) {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        auto stamp = tf2_ros::fromMsg(laser_scan->header.stamp);
        tf2::TimePoint transform_expiration = stamp + transform_tolerance_;
        sendMapToOdomTransform(transform_expiration);
        sent_first_transform_ = true;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "No pose!");
    }
  } else if (latest_tf_valid_) {
    if (tf_broadcast_ == true) {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      tf2::TimePoint transform_expiration = tf2_ros::fromMsg(laser_scan->header.stamp) +
        transform_tolerance_;
      sendMapToOdomTransform(transform_expiration);
    }
  }
}

bool AmclNode::addNewScanner(
  int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::string & laser_scan_frame_id,
  geometry_msgs::msg::PoseStamped & laser_pose)
{
  lasers_.push_back(createLaserObject());
  lasers_update_.push_back(true);
  laser_index = frame_to_laser_.size();

  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = laser_scan_frame_id;
  ident.header.stamp = rclcpp::Time();
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  try {
    tf_buffer_->transform(ident, laser_pose, base_frame_id_);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Couldn't transform from %s to %s, "
      "even though the message notifier is in use",
      laser_scan->header.frame_id.c_str(),
      base_frame_id_.c_str());
    return false;
  }

  pf_vector_t laser_pose_v;
  laser_pose_v.v[0] = laser_pose.pose.position.x;
  laser_pose_v.v[1] = laser_pose.pose.position.y;
  // laser mounting angle gets computed later -> set to 0 here!
  laser_pose_v.v[2] = 0;
  lasers_[laser_index]->SetLaserPose(laser_pose_v);
  frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  return true;
}

bool AmclNode::shouldUpdateFilter(const pf_vector_t pose, pf_vector_t & delta)
{
  delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
  delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
  delta.v[2] = nav2_util::angleutils::angle_diff(pose.v[2], pf_odom_pose_.v[2]);

  // See if we should update the filter
  bool update = fabs(delta.v[0]) > d_thresh_ ||
    fabs(delta.v[1]) > d_thresh_ ||
    fabs(delta.v[2]) > a_thresh_;
  update = update || force_update_;
  return update;
}

bool AmclNode::updateFilter(
  const int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const pf_vector_t & pose)
{
  nav2_util::LaserData ldata;
  ldata.laser = lasers_[laser_index];
  ldata.range_count = laser_scan->ranges.size();
  // To account for lasers that are mounted upside-down, we determine the
  // min, max, and increment angles of the laser in the base frame.
  //
  // Construct min and max angles of laser, in the base_link frame.
  // Here we set the roll pich yaw of the lasers.  We assume roll and pich are zero.
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, laser_scan->angle_min);
  geometry_msgs::msg::QuaternionStamped min_q, inc_q;
  min_q.header.stamp = laser_scan->header.stamp;
  min_q.header.frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  tf2::impl::Converter<false, true>::convert(q, min_q.quaternion);

  q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
  inc_q.header = min_q.header;
  tf2::impl::Converter<false, true>::convert(q, inc_q.quaternion);
  try {
    tf_buffer_->transform(min_q, min_q, base_frame_id_);
    tf_buffer_->transform(inc_q, inc_q, base_frame_id_);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "Unable to transform min/max laser angles into base frame: %s",
      e.what());
    return false;
  }
  double angle_min = tf2::getYaw(min_q.quaternion);
  double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

  // wrapping angle to [-pi .. pi]
  angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

  RCLCPP_DEBUG(
    get_logger(), "Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min,
    angle_increment);

  // Apply range min/max thresholds, if the user supplied them
  if (laser_max_range_ > 0.0) {
    ldata.range_max = std::min(laser_scan->range_max, static_cast<float>(laser_max_range_));
  } else {
    ldata.range_max = laser_scan->range_max;
  }
  double range_min;
  if (laser_min_range_ > 0.0) {
    range_min = std::max(laser_scan->range_min, static_cast<float>(laser_min_range_));
  } else {
    range_min = laser_scan->range_min;
  }

  // The LaserData destructor will free this memory
  ldata.ranges = new double[ldata.range_count][2];
  for (int i = 0; i < ldata.range_count; i++) {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if (laser_scan->ranges[i] <= range_min) {
      ldata.ranges[i][0] = ldata.range_max;
    } else {
      ldata.ranges[i][0] = laser_scan->ranges[i];
    }
    // Compute bearing
    ldata.ranges[i][1] = angle_min +
      (i * angle_increment);
  }
  lasers_[laser_index]->sensorUpdate(pf_, reinterpret_cast<nav2_util::LaserData *>(&ldata));
  lasers_update_[laser_index] = false;
  pf_odom_pose_ = pose;
  return true;
}

void
AmclNode::publishParticleCloud(const pf_sample_set_t * set)
{
  // If initial pose is not known, AMCL does not know the current pose
  if (!initial_pose_is_known_) {return;}
  geometry_msgs::msg::PoseArray cloud_msg;
  cloud_msg.header.stamp = this->now();
  cloud_msg.header.frame_id = global_frame_id_;
  cloud_msg.poses.resize(set->sample_count);
  for (int i = 0; i < set->sample_count; i++) {
    cloud_msg.poses[i].position.x = set->samples[i].pose.v[0];
    cloud_msg.poses[i].position.y = set->samples[i].pose.v[1];
    cloud_msg.poses[i].position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, set->samples[i].pose.v[2]);
    tf2::impl::Converter<false, true>::convert(q, cloud_msg.poses[i].orientation);
  }
  particlecloud_pub_->publish(cloud_msg);
}

bool
AmclNode::getMaxWeightHyp(
  std::vector<amcl_hyp_t> & hyps, amcl_hyp_t & max_weight_hyps,
  int & max_weight_hyp)
{
  // Read out the current hypotheses
  double max_weight = 0.0;
  hyps.resize(pf_->sets[pf_->current_set].cluster_count);
  for (int hyp_count = 0;
    hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
  {
    double weight;
    pf_vector_t pose_mean;
    pf_matrix_t pose_cov;
    if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov)) {
      RCLCPP_ERROR(get_logger(), "Couldn't get stats on cluster %d", hyp_count);
      return false;
    }

    hyps[hyp_count].weight = weight;
    hyps[hyp_count].pf_pose_mean = pose_mean;
    hyps[hyp_count].pf_pose_cov = pose_cov;

    if (hyps[hyp_count].weight > max_weight) {
      max_weight = hyps[hyp_count].weight;
      max_weight_hyp = hyp_count;
    }
  }

  if (max_weight > 0.0) {
    RCLCPP_DEBUG(get_logger(), "Max weight pose: %.3f %.3f %.3f",
      hyps[max_weight_hyp].pf_pose_mean.v[0],
      hyps[max_weight_hyp].pf_pose_mean.v[1],
      hyps[max_weight_hyp].pf_pose_mean.v[2]);

    max_weight_hyps = hyps[max_weight_hyp];
    return true;
  }
  return false;
}

void
AmclNode::publishAmclPose(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
{
  amcl_node_ready_ = true;

  // If initial pose is not known, AMCL does not know the current pose
  if (!initial_pose_is_known_) {
    if (checkElapsedTime(2s, last_time_printed_msg_)) {
      RCLCPP_WARN(get_logger(), "ACML cannot publish a pose or update the transform. "
        "Please set the initial pose...");
      last_time_printed_msg_ = now();
    }
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped p;
  // Fill in the header
  p.header.frame_id = global_frame_id_;
  p.header.stamp = laser_scan->header.stamp;
  // Copy in the pose
  p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
  p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
  tf2::Quaternion q;
  q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
  tf2::impl::Converter<false, true>::convert(q, p.pose.pose.orientation);
  // Copy in the covariance, converting from 3-D to 6-D
  pf_sample_set_t * set = pf_->sets + pf_->current_set;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      // p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
      p.pose.covariance[6 * i + j] = set->cov.m[i][j];
    }
  }
  p.pose.covariance[6 * 5 + 5] = set->cov.m[2][2];
  float temp = 0.0;
  for (auto covariance_value : p.pose.covariance) {
    temp += covariance_value;
  }
  temp += p.pose.pose.position.x + p.pose.pose.position.y;
  if (!std::isnan(temp)) {
    RCLCPP_DEBUG(get_logger(), "Publishing pose");
    pose_pub_->publish(p);
    first_pose_sent_ = true;
    last_published_pose_ = p;
  } else {
    RCLCPP_WARN(get_logger(), "AMCL covariance or pose is NaN, likely due to an invalid "
      "configuration or faulty sensor measurements! Pose is not available!");
  }

  RCLCPP_DEBUG(get_logger(), "New pose: %6.3f %6.3f %6.3f",
    hyps[max_weight_hyp].pf_pose_mean.v[0],
    hyps[max_weight_hyp].pf_pose_mean.v[1],
    hyps[max_weight_hyp].pf_pose_mean.v[2]);
}

void
AmclNode::calculateMaptoOdomTransform(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
{
  // subtracting base to odom from map to base and send map to odom instead
  geometry_msgs::msg::PoseStamped odom_to_map;
  try {
    tf2::Quaternion q;
    q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
    tf2::Transform tmp_tf(q, tf2::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
      hyps[max_weight_hyp].pf_pose_mean.v[1],
      0.0));

    geometry_msgs::msg::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = base_frame_id_;
    tmp_tf_stamped.header.stamp = laser_scan->header.stamp;
    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    tf_buffer_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
  } catch (tf2::TransformException) {
    RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform");
    return;
  }

  tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
  latest_tf_valid_ = true;
}

void
AmclNode::sendMapToOdomTransform(const tf2::TimePoint & transform_expiration)
{
  // AMCL will update transform only when it has knowledge about robot's initial position
  if (!initial_pose_is_known_) {return;}
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = global_frame_id_;
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
  tmp_tf_stamped.child_frame_id = odom_frame_id_;
  tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
  tf_broadcaster_->sendTransform(tmp_tf_stamped);
}

nav2_util::Laser *
AmclNode::createLaserObject()
{
  RCLCPP_INFO(get_logger(), "createLaserObject");

  if (sensor_model_type_ == "beam") {
    return new nav2_util::BeamModel(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_,
             0.0, max_beams_, map_);
  }

  if (sensor_model_type_ == "likelihood_field_prob") {
    return new nav2_util::LikelihoodFieldModelProb(z_hit_, z_rand_, sigma_hit_,
             laser_likelihood_max_dist_, do_beamskip_, beam_skip_distance_, beam_skip_threshold_,
             beam_skip_error_threshold_, max_beams_, map_);
  }

  return new nav2_util::LikelihoodFieldModel(z_hit_, z_rand_, sigma_hit_,
           laser_likelihood_max_dist_, max_beams_, map_);
}

void
AmclNode::initParameters()
{
  double save_pose_rate;
  double tmp_tol;

  get_parameter("alpha1", alpha1_);
  get_parameter("alpha2", alpha2_);
  get_parameter("alpha3", alpha3_);
  get_parameter("alpha4", alpha4_);
  get_parameter("alpha5", alpha5_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("beam_skip_distance", beam_skip_distance_);
  get_parameter("beam_skip_error_threshold", beam_skip_error_threshold_);
  get_parameter("beam_skip_threshold", beam_skip_threshold_);
  get_parameter("do_beamskip", do_beamskip_);
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("lambda_short", lambda_short_);
  get_parameter("laser_likelihood_max_dist", laser_likelihood_max_dist_);
  get_parameter("laser_max_range", laser_max_range_);
  get_parameter("laser_min_range", laser_min_range_);
  get_parameter("laser_model_type", sensor_model_type_);
  get_parameter("max_beams", max_beams_);
  get_parameter("max_particles", max_particles_);
  get_parameter("min_particles", min_particles_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("pf_err", pf_err_);
  get_parameter("pf_z", pf_z_);
  get_parameter("recovery_alpha_fast", alpha_fast_);
  get_parameter("recovery_alpha_slow", alpha_slow_);
  get_parameter("resample_interval", resample_interval_);
  get_parameter("robot_model_type", robot_model_type_);
  get_parameter("save_pose_rate", save_pose_rate);
  get_parameter("sigma_hit", sigma_hit_);
  get_parameter("tf_broadcast", tf_broadcast_);
  get_parameter("transform_tolerance", tmp_tol);
  get_parameter("update_min_a", a_thresh_);
  get_parameter("update_min_d", d_thresh_);
  get_parameter("z_hit", z_hit_);
  get_parameter("z_max", z_max_);
  get_parameter("z_rand", z_rand_);
  get_parameter("z_short", z_short_);

  save_pose_period_ = tf2::durationFromSec(1.0 / save_pose_rate);
  transform_tolerance_ = tf2::durationFromSec(tmp_tol);

  odom_frame_id_ = nav2_util::strip_leading_slash(odom_frame_id_);
  base_frame_id_ = nav2_util::strip_leading_slash(base_frame_id_);
  global_frame_id_ = nav2_util::strip_leading_slash(global_frame_id_);

  last_time_printed_msg_ = now();

  // Semantic checks

  if (min_particles_ > max_particles_) {
    RCLCPP_WARN(get_logger(), "You've set min_particles to be greater than max particles,"
      " this isn't allowed so max_particles will be set to min_particles.");
    max_particles_ = min_particles_;
  }
}

void
AmclNode::initMap()
{
  RCLCPP_INFO(get_logger(), "Requesting map from the map service");

  // Get the map from the map server
  nav_msgs::msg::OccupancyGrid msg;
  if (!map_client_.getMap(msg)) {
    throw "Failed to get map from the map server";
  }

  RCLCPP_INFO(get_logger(), "Received a %dx%d map @ %.3f m/pix",
    msg.info.width, msg.info.height, msg.info.resolution);

  // Perform a check on the map's frame_id
  if (msg.header.frame_id != global_frame_id_) {
    RCLCPP_WARN(get_logger(), "Frame_id of map received:'%s' doesn't match global_frame_id:'%s'."
      " This could cause issues with reading published topics",
      msg.header.frame_id.c_str(), global_frame_id_.c_str());
  }

  // Convert to our own local data structure
  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  // Index of free space
  for (int i = 0; i < map_->size_x; i++) {
    for (int j = 0; j < map_->size_y; j++) {
      if (map_->cells[MAP_INDEX(map_, i, j)].occ_state == -1) {
        free_space_indices.push_back(std::make_pair(i, j));
      }
    }
  }
#endif
}

// Convert an OccupancyGrid map message into the internal representation. This function
// allocates a map_t and returns it.
map_t *
AmclNode::convertMap(const nav_msgs::msg::OccupancyGrid & map_msg)
{
  map_t * map = map_alloc();

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

  map->cells =
    reinterpret_cast<map_cell_t *>(malloc(sizeof(map_cell_t) * map->size_x * map->size_y));

  // Convert to player format
  for (int i = 0; i < map->size_x * map->size_y; i++) {
    if (map_msg.data[i] == 0) {
      map->cells[i].occ_state = -1;
    } else if (map_msg.data[i] == 100) {
      map->cells[i].occ_state = +1;
    } else {
      map->cells[i].occ_state = 0;
    }
  }

  return map;
}

void
AmclNode::initTransforms()
{
  RCLCPP_INFO(get_logger(), "initTransforms");

  // Initialize transform listener and broadcaster
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, rclcpp_node_, false);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);

  sent_first_transform_ = false;
  latest_tf_valid_ = false;
  memset(&latest_tf_, 0, sizeof(latest_tf_));
}

void
AmclNode::initMessageFilters()
{
  laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
    rclcpp_node_.get(), scan_topic_, rmw_qos_profile_sensor_data);

  laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10, rclcpp_node_);

  laser_scan_connection_ = laser_scan_filter_->registerCallback(std::bind(&AmclNode::laserReceived,
      this, std::placeholders::_1));
}

void
AmclNode::initPubSub()
{
  RCLCPP_INFO(get_logger(), "initPubSub");

  particlecloud_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud",
      rclcpp::SensorDataQoS());

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&AmclNode::initialPoseReceived, this, std::placeholders::_1));
}

void
AmclNode::initServices()
{
  global_loc_srv_ = create_service<std_srvs::srv::Empty>("global_localization",
      std::bind(&AmclNode::globalLocalizationCallback, this, _1, _2, _3));

  nomotion_update_srv_ = create_service<std_srvs::srv::Empty>("request_nomotion_update",
      std::bind(&AmclNode::nomotionUpdateCallback, this, _1, _2, _3));
}

void
AmclNode::initOdometry()
{
  // TODO(mjeronimo): We should handle persistance of the last known pose of the robot. We could
  // then read that pose here and initialize using that.

  // When pausing and resuming, remember the last robot pose so we don't start at 0:0 again
  init_pose_[0] = last_published_pose_.pose.pose.position.x;
  init_pose_[1] = last_published_pose_.pose.pose.position.y;
  init_pose_[2] = last_published_pose_.pose.pose.position.z;

  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);

  motion_model_ = std::unique_ptr<nav2_util::MotionModel>(nav2_util::MotionModel::createMotionModel(
        robot_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_));

  memset(&latest_odom_pose_, 0, sizeof(latest_odom_pose_));
}

void
AmclNode::initParticleFilter()
{
  // Create the particle filter
  pf_ = pf_alloc(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
      (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
      reinterpret_cast<void *>(map_));
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];

  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];

  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);

  pf_init_ = false;
  resample_count_ = 0;
  memset(&pf_odom_pose_, 0, sizeof(pf_odom_pose_));
}

void
AmclNode::initLaserScan()
{
  scan_error_count_ = 0;
  memset(&last_laser_received_ts_, 0, sizeof(last_laser_received_ts_));
}

}  // namespace nav2_amcl
