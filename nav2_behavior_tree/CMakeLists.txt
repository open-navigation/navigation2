cmake_minimum_required(VERSION 3.5)
project(nav2_behavior_tree CXX)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${library_name} PUBLIC
  ${action_msgs_TARGETS}
  behaviortree_cpp::behaviortree_cpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)

add_library(nav2_compute_path_to_pose_action_bt_node SHARED plugins/action/compute_path_to_pose_action.cpp)
list(APPEND plugin_libs nav2_compute_path_to_pose_action_bt_node)

add_library(nav2_compute_path_through_poses_action_bt_node SHARED plugins/action/compute_path_through_poses_action.cpp)
list(APPEND plugin_libs nav2_compute_path_through_poses_action_bt_node)

add_library(nav2_controller_cancel_bt_node SHARED plugins/action/controller_cancel_node.cpp)
list(APPEND plugin_libs nav2_controller_cancel_bt_node)

add_library(nav2_wait_cancel_bt_node SHARED plugins/action/wait_cancel_node.cpp)
list(APPEND plugin_libs nav2_wait_cancel_bt_node)

add_library(nav2_spin_cancel_bt_node SHARED plugins/action/spin_cancel_node.cpp)
list(APPEND plugin_libs nav2_spin_cancel_bt_node)

add_library(nav2_back_up_cancel_bt_node SHARED plugins/action/back_up_cancel_node.cpp)
list(APPEND plugin_libs nav2_back_up_cancel_bt_node)

add_library(nav2_assisted_teleop_cancel_bt_node SHARED plugins/action/assisted_teleop_cancel_node.cpp)
list(APPEND plugin_libs nav2_assisted_teleop_cancel_bt_node)

add_library(nav2_drive_on_heading_cancel_bt_node SHARED plugins/action/drive_on_heading_cancel_node.cpp)
list(APPEND plugin_libs nav2_drive_on_heading_cancel_bt_node)

add_library(nav2_smooth_path_action_bt_node SHARED plugins/action/smooth_path_action.cpp)
list(APPEND plugin_libs nav2_smooth_path_action_bt_node)

add_library(nav2_follow_path_action_bt_node SHARED plugins/action/follow_path_action.cpp)
list(APPEND plugin_libs nav2_follow_path_action_bt_node)

add_library(nav2_back_up_action_bt_node SHARED plugins/action/back_up_action.cpp)
list(APPEND plugin_libs nav2_back_up_action_bt_node)

add_library(nav2_drive_on_heading_bt_node SHARED plugins/action/drive_on_heading_action.cpp)
list(APPEND plugin_libs nav2_drive_on_heading_bt_node)

add_library(nav2_spin_action_bt_node SHARED plugins/action/spin_action.cpp)
list(APPEND plugin_libs nav2_spin_action_bt_node)

add_library(nav2_wait_action_bt_node SHARED plugins/action/wait_action.cpp)
list(APPEND plugin_libs nav2_wait_action_bt_node)

add_library(nav2_assisted_teleop_action_bt_node SHARED plugins/action/assisted_teleop_action.cpp)
list(APPEND plugin_libs nav2_assisted_teleop_action_bt_node)

add_library(nav2_clear_costmap_service_bt_node SHARED plugins/action/clear_costmap_service.cpp)
list(APPEND plugin_libs nav2_clear_costmap_service_bt_node)

add_library(nav2_is_stuck_condition_bt_node SHARED plugins/condition/is_stuck_condition.cpp)
list(APPEND plugin_libs nav2_is_stuck_condition_bt_node)

add_library(nav2_transform_available_condition_bt_node SHARED plugins/condition/transform_available_condition.cpp)
list(APPEND plugin_libs nav2_transform_available_condition_bt_node)

add_library(nav2_goal_reached_condition_bt_node SHARED plugins/condition/goal_reached_condition.cpp)
list(APPEND plugin_libs nav2_goal_reached_condition_bt_node)

add_library(nav2_globally_updated_goal_condition_bt_node SHARED plugins/condition/globally_updated_goal_condition.cpp)
list(APPEND plugin_libs nav2_globally_updated_goal_condition_bt_node)

add_library(nav2_goal_updated_condition_bt_node SHARED plugins/condition/goal_updated_condition.cpp)
list(APPEND plugin_libs nav2_goal_updated_condition_bt_node)

add_library(nav2_is_path_valid_condition_bt_node SHARED plugins/condition/is_path_valid_condition.cpp)
list(APPEND plugin_libs nav2_is_path_valid_condition_bt_node)

add_library(nav2_time_expired_condition_bt_node SHARED plugins/condition/time_expired_condition.cpp)
list(APPEND plugin_libs nav2_time_expired_condition_bt_node)

add_library(nav2_path_expiring_timer_condition SHARED plugins/condition/path_expiring_timer_condition.cpp)
list(APPEND plugin_libs nav2_path_expiring_timer_condition)

add_library(nav2_distance_traveled_condition_bt_node SHARED plugins/condition/distance_traveled_condition.cpp)
list(APPEND plugin_libs nav2_distance_traveled_condition_bt_node)

add_library(nav2_initial_pose_received_condition_bt_node SHARED plugins/condition/initial_pose_received_condition.cpp)
list(APPEND plugin_libs nav2_initial_pose_received_condition_bt_node)

add_library(nav2_is_battery_charging_condition_bt_node SHARED plugins/condition/is_battery_charging_condition.cpp)
list(APPEND plugin_libs nav2_is_battery_charging_condition_bt_node)

add_library(nav2_is_battery_low_condition_bt_node SHARED plugins/condition/is_battery_low_condition.cpp)
list(APPEND plugin_libs nav2_is_battery_low_condition_bt_node)

add_library(nav2_are_error_codes_active_condition_bt_node SHARED plugins/condition/are_error_codes_present_condition.cpp)
list(APPEND plugin_libs nav2_are_error_codes_active_condition_bt_node)

add_library(nav2_would_a_controller_recovery_help_condition_bt_node SHARED plugins/condition/would_a_controller_recovery_help_condition.cpp)
list(APPEND plugin_libs nav2_would_a_controller_recovery_help_condition_bt_node)

add_library(nav2_would_a_planner_recovery_help_condition_bt_node SHARED plugins/condition/would_a_planner_recovery_help_condition.cpp)
list(APPEND plugin_libs nav2_would_a_planner_recovery_help_condition_bt_node)

add_library(nav2_would_a_smoother_recovery_help_condition_bt_node SHARED plugins/condition/would_a_smoother_recovery_help_condition.cpp)
list(APPEND plugin_libs nav2_would_a_smoother_recovery_help_condition_bt_node)

add_library(nav2_reinitialize_global_localization_service_bt_node SHARED plugins/action/reinitialize_global_localization_service.cpp)
list(APPEND plugin_libs nav2_reinitialize_global_localization_service_bt_node)

add_library(nav2_rate_controller_bt_node SHARED plugins/decorator/rate_controller.cpp)
list(APPEND plugin_libs nav2_rate_controller_bt_node)

add_library(nav2_distance_controller_bt_node SHARED plugins/decorator/distance_controller.cpp)
list(APPEND plugin_libs nav2_distance_controller_bt_node)

add_library(nav2_speed_controller_bt_node SHARED plugins/decorator/speed_controller.cpp)
list(APPEND plugin_libs nav2_speed_controller_bt_node)

add_library(nav2_truncate_path_action_bt_node SHARED plugins/action/truncate_path_action.cpp)
list(APPEND plugin_libs nav2_truncate_path_action_bt_node)

add_library(nav2_truncate_path_local_action_bt_node SHARED plugins/action/truncate_path_local_action.cpp)
list(APPEND plugin_libs nav2_truncate_path_local_action_bt_node)

add_library(nav2_goal_updater_node_bt_node SHARED plugins/decorator/goal_updater_node.cpp)
list(APPEND plugin_libs nav2_goal_updater_node_bt_node)

add_library(nav2_path_longer_on_approach_bt_node SHARED plugins/decorator/path_longer_on_approach.cpp)
list(APPEND plugin_libs nav2_path_longer_on_approach_bt_node)

add_library(nav2_recovery_node_bt_node SHARED plugins/control/recovery_node.cpp)
list(APPEND plugin_libs nav2_recovery_node_bt_node)

add_library(nav2_navigate_to_pose_action_bt_node SHARED plugins/action/navigate_to_pose_action.cpp)
list(APPEND plugin_libs nav2_navigate_to_pose_action_bt_node)

add_library(nav2_navigate_through_poses_action_bt_node SHARED plugins/action/navigate_through_poses_action.cpp)
list(APPEND plugin_libs nav2_navigate_through_poses_action_bt_node)

add_library(nav2_remove_passed_goals_action_bt_node SHARED plugins/action/remove_passed_goals_action.cpp)
list(APPEND plugin_libs nav2_remove_passed_goals_action_bt_node)

add_library(nav2_pipeline_sequence_bt_node SHARED plugins/control/pipeline_sequence.cpp)
list(APPEND plugin_libs nav2_pipeline_sequence_bt_node)

add_library(nav2_round_robin_node_bt_node SHARED plugins/control/round_robin_node.cpp)
list(APPEND plugin_libs nav2_round_robin_node_bt_node)

add_library(nav2_single_trigger_bt_node SHARED plugins/decorator/single_trigger_node.cpp)
list(APPEND plugin_libs nav2_single_trigger_bt_node)

add_library(nav2_planner_selector_bt_node SHARED plugins/action/planner_selector_node.cpp)
list(APPEND plugin_libs nav2_planner_selector_bt_node)

add_library(nav2_controller_selector_bt_node SHARED plugins/action/controller_selector_node.cpp)
list(APPEND plugin_libs nav2_controller_selector_bt_node)

add_library(nav2_smoother_selector_bt_node SHARED plugins/action/smoother_selector_node.cpp)
list(APPEND plugin_libs nav2_smoother_selector_bt_node)

add_library(nav2_goal_checker_selector_bt_node SHARED plugins/action/goal_checker_selector_node.cpp)
list(APPEND plugin_libs nav2_goal_checker_selector_bt_node)

add_library(nav2_progress_checker_selector_bt_node SHARED plugins/action/progress_checker_selector_node.cpp)
list(APPEND plugin_libs nav2_progress_checker_selector_bt_node)

add_library(nav2_goal_updated_controller_bt_node SHARED plugins/decorator/goal_updated_controller.cpp)
list(APPEND plugin_libs nav2_goal_updated_controller_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin}
    PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${bt_plugin} PUBLIC
    behaviortree_cpp::behaviortree_cpp
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
    nav2_util::nav2_util_core
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_lifecycle::rclcpp_lifecycle
    ${sensor_msgs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
  )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name} ${plugin_libs}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# we will embed the list of plugin names inside a header file
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
configure_file(plugins_list.hpp.in ${GENERATED_DIR}/plugins_list.hpp)

add_executable(generate_nav2_tree_nodes_xml src/generate_nav2_tree_nodes_xml.cpp)
target_link_libraries(generate_nav2_tree_nodes_xml PRIVATE
  behaviortree_cpp::behaviortree_cpp
  nav2_util::nav2_util_core
)
# allow generate_nav2_tree_nodes_xml to find plugins_list.hpp
target_include_directories(generate_nav2_tree_nodes_xml PRIVATE ${GENERATED_DIR})
install(TARGETS generate_nav2_tree_nodes_xml DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
install(FILES ${GENERATED_DIR}/plugins_list.hpp DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_find_gtest()

  add_subdirectory(test)
endif()

ament_export_include_directories(
  include/${PROJECT_NAME}
)

ament_export_libraries(${library_name})

ament_export_dependencies(
  action_msgs
  behaviortree_cpp
  geometry_msgs
  nav2_common
  nav2_msgs
  nav2_util
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)
ament_export_targets(${library_name})

ament_package()
