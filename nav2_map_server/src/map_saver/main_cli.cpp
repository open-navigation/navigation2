// Copyright 2020 Shivam Pandey pandeyshvivamm2017robotics@gmail.com
// Copyright 2019 Rover Robotics
// Copyright (c) 2008, Willow Garage, Inc.
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
//
// Created by shivam on 10/3/20.
//

#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include "nav2_map_server/map_2d/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace nav2_map_server;  // NOLINT

const char * USAGE_STRING{
    "Usage:\n"
    "  map_saver_cli [arguments] [--ros-args ROS remapping args]\n"
    "\n"
    "Arguments:\n"
    "  -h/--help\n"
    "  -t <map_topic>\n"
    "  -f <mapname>\n"
    "  --occ <threshold_occupied>\n"
    "  --free <threshold_free>\n"
    "  --fmt <map_format>\n"
    "  --as_bin (for pointClouds)Give the flag to save map with binary encodings\n"
    "  --origin <[size 7 vector of floats]>\n"
    "  --mode trinary(default)/scale/raw\n"
    "\n"
    "NOTE: --ros-args should be passed at the end of command line"};

typedef enum
{
  COMMAND_MAP_TOPIC,
  COMMAND_MAP_FILE_NAME,
  COMMAND_IMAGE_FORMAT,
  COMMAND_OCCUPIED_THRESH,
  COMMAND_FREE_THRESH,
  COMMAND_MODE,
  COMMAND_ENCODING,
  COMMAND_VIEW_POINT
} COMMAND_TYPE;

struct cmd_struct
{
  const char * cmd;
  COMMAND_TYPE command_type;
};

typedef enum
{
  ARGUMENTS_INVALID,
  ARGUMENTS_VALID,
  HELP_MESSAGE
} ARGUMENTS_STATUS;

struct SaveParamList{
  map_2d::SaveParameters save_parameters_2d;
  map_3d::SaveParameters save_parameters_3d;
};

// Arguments parser
// Input parameters: logger, argc, argv
// Output parameters: map_topic, save_parameters
ARGUMENTS_STATUS parse_arguments(
    const rclcpp::Logger & logger, int argc, char ** argv,
    std::string & map_topic, SaveParamList & save_parameters)
{
  const struct cmd_struct commands[] = {
      {"-t", COMMAND_MAP_TOPIC},
      {"-f", COMMAND_MAP_FILE_NAME},
      {"--occ", COMMAND_OCCUPIED_THRESH},
      {"--free", COMMAND_FREE_THRESH},
      {"--mode", COMMAND_MODE},
      {"--as_bin", COMMAND_ENCODING},
      {"--origin", COMMAND_VIEW_POINT},
      {"--fmt", COMMAND_IMAGE_FORMAT}
  };

  std::vector<std::string> arguments(argv + 1, argv + argc);
  std::vector<rclcpp::Parameter> params_from_args;


  size_t cmd_size = sizeof(commands) / sizeof(commands[0]);
  size_t i;
  for (auto it = arguments.begin(); it != arguments.end(); it++) {
    if (*it == "-h" || *it == "--help") {
      std::cout << USAGE_STRING << std::endl;
      return HELP_MESSAGE;
    }
    if (*it == "--ros-args") {
      break;
    }
    for (i = 0; i < cmd_size; i++) {
      if (commands[i].cmd == *it) {
        if ((it + 1) == arguments.end()) {
          RCLCPP_ERROR(logger, "Wrong argument: %s should be followed by a value.", it->c_str());
          return ARGUMENTS_INVALID;
        }
        it++;
        switch (commands[i].command_type) {
          case COMMAND_MAP_TOPIC:
            map_topic = *it;
            break;
          case COMMAND_MAP_FILE_NAME:
            save_parameters.save_parameters_2d.map_file_name = *it;
            save_parameters.save_parameters_3d.map_file_name = *it;
            break;
          case COMMAND_FREE_THRESH:
            save_parameters.save_parameters_2d.free_thresh = atoi(it->c_str());
            break;
          case COMMAND_OCCUPIED_THRESH:
            save_parameters.save_parameters_2d.occupied_thresh = atoi(it->c_str());
            break;
          case COMMAND_IMAGE_FORMAT:
            save_parameters.save_parameters_2d.image_format = *it;
            save_parameters.save_parameters_3d.format = *it;
            break;
          case COMMAND_MODE:
            try {
              save_parameters.save_parameters_2d.mode = map_2d::map_mode_from_string(*it);
            } catch (std::invalid_argument &) {
              save_parameters.save_parameters_2d.mode = map_2d::MapMode::Trinary;
              RCLCPP_WARN(
                  logger,
                  "Map mode parameter not recognized: %s, using default value (trinary)",
                  it->c_str());
            }
            break;
          case COMMAND_VIEW_POINT:
            for (int k = 0; k < 9; ++k) {
              std::string tmp = *it;
              if (tmp == "[") {
                it++;
                continue;
              } else if (tmp == "]") {
                break;
              }
              if (k < 4) {
                save_parameters.save_parameters_3d.origin.center.push_back(
                    std::stof(tmp.substr(0, tmp.size() - 1)));
              } else if (k < 7) {
                save_parameters.save_parameters_3d.origin.orientation.push_back(
                    std::stof(tmp.substr(0, tmp.size() - 1)));
              } else if (k == 7) {
                save_parameters.save_parameters_3d.origin.orientation.push_back(std::stof(tmp));
              }
              it++;
            }
            break;
          case COMMAND_ENCODING:
            it--;  // as this one is a simple flag that puts binary format to on
            save_parameters.save_parameters_3d.as_binary = true;
            break;
        }
        break;
      }
    }
    if (i == cmd_size) {
      RCLCPP_ERROR(logger, "Wrong argument: %s", it->c_str());
      return ARGUMENTS_INVALID;
    }
  }

  return ARGUMENTS_VALID;
}

int main(int argc, char ** argv)
{
  // ROS2 init
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("map_saver_cli");

  // Parse CLI-arguments
  SaveParamList save_parameters;
  std::string map_topic = "map";
  switch (parse_arguments(logger, argc, argv, map_topic, save_parameters)) {
    case ARGUMENTS_INVALID:
      rclcpp::shutdown();
      return -1;
    case HELP_MESSAGE:
      rclcpp::shutdown();
      return 0;
    case ARGUMENTS_VALID:
      break;
  }

  // Call saveMapTopicToFile()
  int retcode;
  try {
    if (save_parameters.save_parameters_3d.format == "pcd") {

      auto map_saver = std::make_shared<nav2_map_server::MapSaver<sensor_msgs::msg::PointCloud2>>();
      if (map_saver->saveMapTopicToFile(map_topic, save_parameters.save_parameters_3d)) {
        retcode = 0;
      } else {
        retcode = 1;
      }

    } else {
      auto map_saver = std::make_shared<nav2_map_server::MapSaver<nav_msgs::msg::OccupancyGrid>>();
      if (map_saver->saveMapTopicToFile(map_topic, save_parameters.save_parameters_2d)) {
        retcode = 0;
      } else {
        retcode = 1;
      }
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(logger, "Unexpected problem appear: %s", e.what());
    retcode = -1;
  }

  // Exit
  rclcpp::shutdown();
  return retcode;
}
