#ifndef NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_
#define NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_

// Interfaces
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/interfaces/motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/poseWithCovariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/LaserScan.hpp"
#include "nav_msgs/msg/OccupancyGrid.hpp"

// Others
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{
class DummySolver2d : public nav2_localization::Solver
{
public:
	DummySolver2d(){}

	geometry_msgs::msg::PoseWithCovariance solve(
		const nav_msgs::msg::Odometry& curr_odom,
		const sensor_msgs::msg::LaserScan& scan);

	void configure(
		const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
		SampleMotionModel& motionSampler,
		Matcher2d& matcher,
		const nav_msgs::msg::Odometry& odom,
		const geometry_msgs::msg::Pose& pose);
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_
