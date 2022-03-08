#ifndef NAV2_COSTMAP_2D__RAY_TRACER_HPP_
#define NAV2_COSTMAP_2D__RAY_TRACER_HPP_

#include <vector>

#include "image_geometry/pinhole_camera_model.h"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "opencv2/core.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/convert.h"




namespace nav2_costmap_2d
{
/**
 * @class RayCaster
 * @brief Takes in camera calibration data and traces pixels over a given plane
 */
    
class RayCaster
{
public:
    /**
   * @brief  Constructs an observation buffer
   * @param  camera_info_topic_name The topic in which the camera calibration is published
   * @param  aligned_pc2_topic_name The topic of the image aligned pointcloud2
   * @param  use_pointcloud Whether to use the pointcloud or the camera calibration for ray tracing
   * @param  tf2_buffer A reference to a tf2 Buffer
   * @param  global_frame The frame to transform PointClouds into
   * @param  sensor_frame The frame of the origin of the sensor, can be left blank to be read from the messages
   * @param  tf_tolerance The amount of time to wait for a transform to be available when setting a new global frame
   */
    RayCaster();

    /**
   * @brief  Destructor... cleans up
   */
    ~RayCaster(){};

    void initialize(
        rclcpp::Node::SharedPtr & parent_node,
        std::string camera_info_topic_name,
        std::string aligned_pc2_topic_name,
        bool use_pointcloud,
        tf2_ros::Buffer * tf2_buffer,
        std::string global_frame,
        tf2::Duration tf_tolerance,
        double max_trace_distance
        );

    bool worldToImage(geometry_msgs::msg::PointStamped& point, cv::Point2d& pixel);

    bool imageToGroundPlane(cv::Point2d& pixel, geometry_msgs::msg::PointStamped& point);
    

private:

    geometry_msgs::msg::PointStamped rayToPointStamped(cv::Point3d& ray_end, std::string& frame_id);

    void cameraInfoCb(sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void pointCloudCb(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    rclcpp::Node::SharedPtr parent_node_;
    std::string global_frame_;
    std::string sensor_frame_;
    tf2_ros::Buffer * tf2_buffer_;
    std::string camera_info_topic_name_;
    std::string aligned_pc2_topic_name_;
    tf2::Duration tf_tolerance_;
    bool use_pointcloud_;
    double max_trace_distance_;
    
    bool ready_to_raytrace_ = false;
    
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pc2_sub_;

    sensor_msgs::msg::PointCloud2 pointcloud_msg_;
    image_geometry::PinholeCameraModel camera_model_ = image_geometry::PinholeCameraModel();

    geometry_msgs::msg::PointStamped camera_origin_;

};

} // namespace nav2_costmap_2d



#endif  // NAV2_COSTMAP_2D__RAY_TRACER_HPP_