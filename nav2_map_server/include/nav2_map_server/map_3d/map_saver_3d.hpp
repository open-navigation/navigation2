//
// Created by shivam on 9/24/20.
//

#ifndef NAV2_MAP_SERVER_INCLUDE_MAP_3D_MAP_SAVER_3D_HPP_
#define NAV2_MAP_SERVER_INCLUDE_MAP_3D_MAP_SAVER_3D_HPP_
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/srv/save_map3_d.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "map_3d/map_io_3d.hpp"
#include "nav2_map_server/map_saver_core.hpp"

namespace nav2_map_server
{

/**
 * @class nav2_map_server::MapSaver
 * @brief A class that provides map saving methods and services
 */
template <>
class MapSaver<sensor_msgs::msg::PointCloud2> : public nav2_util::LifecycleNode
{
 public:
  /**
   * @brief Constructor for the nav2_map_server::MapSaver
   * @param is_pcd Bool for distinguishing b/w pcd and image, false by default
   */
  MapSaver();

  /**
   * @brief Destructor for the nav2_map_server::MapServer
   */
  ~MapSaver() override;

  /**
   * @brief Read a message from incoming map topic and save map to a file
   * @param map_topic Incoming map topic name
   * @param save_parameters Map saving parameters.
   * @return true of false
   */
  bool saveMapTopicToFile(
      const std::string & map_topic,
      const map_3d::SaveParameters & save_parameters);

 protected:
  /**
   * @brief Sets up map saving service
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when node switched to active state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when node switched to inactive state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when it is required node clean-up
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when Error is raised
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Map saving service callback
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void saveMapCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<nav2_msgs::srv::SaveMap3D::Request> request,
      std::shared_ptr<nav2_msgs::srv::SaveMap3D::Response> response);

  // The timeout for saving the map in service
  std::shared_ptr<rclcpp::Duration> save_map_timeout_;

  // The name of the service for saving a map from topic
  const std::string save_map_service_name_{"save_map"};
  // A service to save PointCloud2 data to a file at runtime (SaveMap)
  rclcpp::Service<nav2_msgs::srv::SaveMap3D>::SharedPtr save_map_service_;
};

}  // namespace nav2_map_server
#endif //NAV2_MAP_SERVER_INCLUDE_MAP_3D_MAP_SAVER_3D_HPP_
