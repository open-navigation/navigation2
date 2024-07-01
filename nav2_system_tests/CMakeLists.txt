cmake_minimum_required(VERSION 3.5)
project(nav2_system_tests)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(navigation2)
find_package(angles REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_minimal_tb3_sim REQUIRED)

nav2_package()

set(dependencies
  rclcpp
  nav2_util
  nav2_map_server
  nav2_msgs
  nav_msgs
  visualization_msgs
  nav2_amcl
  nav2_lifecycle_manager
  nav2_behavior_tree
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  rclpy
  nav2_planner
  nav2_navfn_planner
  angles
  behaviortree_cpp
  pluginlib
)

set(local_controller_plugin_lib local_controller)

add_library(${local_controller_plugin_lib} SHARED src/error_codes/controller/controller_error_plugins.cpp)
ament_target_dependencies(${local_controller_plugin_lib} ${dependencies})
target_compile_definitions(${local_controller_plugin_lib} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core src/error_codes/controller_plugins.xml)

install(TARGETS ${local_controller_plugin_lib}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES src/error_codes/controller_plugins.xml
    DESTINATION share/${PROJECT_NAME}
)

set(global_planner_plugin_lib global_planner)

add_library(${global_planner_plugin_lib} SHARED src/error_codes/planner/planner_error_plugin.cpp)
ament_target_dependencies(${global_planner_plugin_lib} ${dependencies})
target_compile_definitions(${global_planner_plugin_lib} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core src/error_codes/planner_plugins.xml)

install(TARGETS ${global_planner_plugin_lib}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES src/error_codes/planner_plugins.xml
    DESTINATION share/${PROJECT_NAME}
)

set(smoother_plugin_lib smoother)

add_library(${smoother_plugin_lib} SHARED src/error_codes/smoother/smoother_error_plugin.cpp)
ament_target_dependencies(${smoother_plugin_lib} ${dependencies})
target_compile_definitions(${smoother_plugin_lib} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core src/error_codes/smoother_plugins.xml)

install(TARGETS ${smoother_plugin_lib}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES src/error_codes/smoother_plugins.xml
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  add_subdirectory(src/behavior_tree)
  add_subdirectory(src/planning)
  add_subdirectory(src/localization)
  add_subdirectory(src/system)
  add_subdirectory(src/system_failure)
  add_subdirectory(src/updown)
  add_subdirectory(src/waypoint_follower)
  # Uncomment after https://github.com/ros-navigation/navigation2/pull/3634
  # add_subdirectory(src/gps_navigation)
  add_subdirectory(src/behaviors/spin)
  # add_subdirectory(src/behaviors/wait)
  # add_subdirectory(src/behaviors/backup)
  add_subdirectory(src/behaviors/drive_on_heading)
  add_subdirectory(src/behaviors/assisted_teleop)
  add_subdirectory(src/costmap_filters)
  add_subdirectory(src/error_codes)
  install(DIRECTORY maps models DESTINATION share/${PROJECT_NAME})

endif()

ament_export_libraries(${local_controller_plugin_lib})
ament_export_libraries(${global_planner_plugin_lib})
ament_export_libraries(${smoother_plugin_lib})
ament_package()
