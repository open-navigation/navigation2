#ifndef NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_
#define NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{

/**
 * @class Matcher2d
 * @brief Abstract interface for a 2D matcher for localization purposes to adhere to with pluginlib
 */
class Matcher2d
{
public:
    Matcher2d(){}

	using Ptr = std::shared_ptr<nav2_localization::Matcher2d>;

    /**
     * @brief Computes how likely is it to read a measurement given a position in a map
     * @param scan 2D laser-like measurement
     * @param pose Hypothesis on the robot's pose
     * @return Probability of how likely is it to perceive the inputted measurement assuming that the robot is in the given pose inside the provided map
     */
	virtual float match(
		const sensor_msgs::msg::LaserScan& scan,
		const geometry_msgs::msg::Pose& pose) = 0;

	virtual void configure(
		const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
		nav_msgs::msg::OccupancyGrid& map) = 0;

	virtual void activate() = 0;

	virtual void deactivate() = 0;

	virtual void cleanup() = 0;

protected:
	rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
	nav_msgs::msg::OccupancyGrid& map_; // Reference to the 2D occupancy grid map of the environment where the robot is
};  
} // nav2_localization

#endif // NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_
