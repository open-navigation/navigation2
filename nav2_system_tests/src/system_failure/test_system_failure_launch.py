#!/usr/bin/env python3

# Copyright (c) 2018 Intel Corporation
# Copyright (c) 2020 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from os import environ
from os import pathsep

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService

from nav2_common.launch import RewrittenYaml

from scripts import GazeboRosPaths


def generate_launch_description():
    model, plugin, media = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    aws_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')

    env = {'GAZEBO_MODEL_PATH': model,
           'GAZEBO_PLUGIN_PATH': plugin,
           'GAZEBO_RESOURCE_PATH': media}

    map_yaml_file = os.path.join(aws_dir, 'maps', '005', 'map.yaml')

    bt_navigator_xml = os.path.join(get_package_share_directory('nav2_bt_navigator'),
                                    'behavior_trees',
                                    os.getenv('BT_NAVIGATOR_XML'))

    bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')

    # Replace the `use_astar` setting on the params file
    param_substitutions = {
        'planner_server.ros__parameters.GridBased.use_astar': 'False'}
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    context = LaunchContext()
    new_yaml = configured_params.perform(context)

    world = LaunchConfiguration('world')

    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {'x': LaunchConfiguration('x_pose', default='1.80'),
            'y': LaunchConfiguration('y_pose', default='2.20'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),

        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(aws_dir, 'worlds', 'no_roof_small_warehouse',
                                       'no_roof_small_warehouse.world'),
            description='Full path to world model file to load'),

        # Launch gazebo server for simulation
        ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                 '--minimal_comms', world],
            additional_env=env,
            cwd=[aws_dir], output='screen'),

        DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(bringup_dir, 'worlds', 'waffle.model'),
            description='Full path to robot sdf file to spawn the robot in gazebo'),

        DeclareLaunchArgument(
            'robot_name',
            default_value='turtlebot3_waffle',
            description='name of the robot'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', robot_name,
                '-file', robot_sdf,
                '-robot_namespace', '',
                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]),
        # TODO(orduno) Launch the robot state publisher instead
        #              using a local copy of TB3 urdf file
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_scan']),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={'namespace': '',
                              'use_namespace': 'False',
                              'map': map_yaml_file,
                              'use_sim_time': 'True',
                              'params_file': new_yaml,
                              'bt_xml_file': bt_navigator_xml,
                              'use_composition': 'False',
                              'autostart': 'True'}.items()),
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test1_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'tester_node.py'),
             '-r', '0.0', '0.0', '100.0', '100.0'],
        name='tester_node',
        output='screen')

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
