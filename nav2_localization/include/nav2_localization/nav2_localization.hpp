#ifndef NAV2_LOCALIZATION__NAV2_LOCALIZATION_HPP_
#define NAV2_LOCALIZATION__NAV2_LOCALIZATION_HPP_

#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "message_filters/subscriber.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_localization
{

class LocalizationServer : public nav2_util::LifecycleNode
{
public:

    /**
    * @brief Constructor for nav2_localization::LocalizationServer
    */
    LocalizationServer();
    /**
    * @brief Destructor for nav2_localization::LocalizationServer
    */
    ~LocalizationServer();

protected:
    /**
     * @brief Configures server parameters and member variables
     *
     * Configures motion model and matcher plugins; Initialize odom subscriber.
     * @param state LifeCycle Node's state
     * @return Success or Failure
     * @throw pluginlib::PluginlibException When failed to initialize motion
     * model or matcher plugins
     */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Activates member variables
     *
     * Activates motion model and matcher.
     * @param state LifeCycle Node's state
     * @return Success or Failure
     */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Deactivates member variables
     *
     * @param state LifeCycle Node's state
     * @return Success or Failure
     */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Calls clean up states and resets member variables.
     *
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
    /**
     * @brief Called when in Error state
     * @param state LifeCycle Node's state
     * @return Success or Failure
     */
    nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

    void initParameters();
    void initPubSub();

    // Map
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr map_sub_;
    void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    bool first_map_only_{true};
    bool first_map_received_{false};
    nav_msgs::msg::OccupancyGrid map_;

    // Laser scan
    std::string scan_topic_;
    void laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan);

    // Transforms
    void initTransforms();
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Odometry
    std::string odom_frame_id_;
    std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
    
    // Message filters
    void initMessageFilters();
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser_scan_sub_;
    std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
    message_filters::Connection laser_scan_connection_;

    // Sample Motion Model Plugin
    pluginlib::ClassLoader<nav2_localization_base::SampleMotionModel> sample_motion_model_loader_;
    nav2_localization_base::SampleMotionModel::Ptr sample_motion_model_;
    std::string default_sample_motion_model_id_;
    std::string sample_motion_model_id_;
    std::string sample_motion_model_type_;

    // Matcher Plugin
    // TODO

};

}

#endif // NAV2_LOCALIZATION__NAV2_LOCALIZATION_HPP_