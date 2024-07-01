# Copyright (c) 2024 Open Navigation LLC
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

from math import acos, cos, sin
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Twist
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster


# This test can be run standalone with:
# python3 -u -m pytest test_docking_server.py -s

@pytest.mark.rostest
def generate_test_description():

    return LaunchDescription([
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            parameters=[{'wait_charge_timeout': 1.0,
                         'dock_plugins': ['test_dock_plugin'],
                         'test_dock_plugin': {
                            'plugin': 'opennav_docking::SimpleChargingDock',
                            'use_battery_status': True},
                         'docks': ['test_dock'],
                         'test_dock': {
                            'type': 'test_dock_plugin',
                            'frame': 'odom',
                            'pose': [10.0, 0.0, 0.0]
                         }}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['docking_server']}]
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestDockingServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Latest odom -> base_link
        cls.x = 0.0
        cls.y = 0.0
        cls.theta = 0.0
        # Track charge state
        cls.is_charging = False
        # Latest command velocity
        cls.command = Twist()
        cls.node = rclpy.create_node('test_docking_server')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def command_velocity_callback(self, msg):
        self.node.get_logger().info('Command: %f %f' % (msg.linear.x, msg.angular.z))
        self.command = msg

    def timer_callback(self):
        # Propagate command
        period = 0.05
        self.x += cos(self.theta) * self.command.linear.x * period
        self.y += sin(self.theta) * self.command.linear.x * period
        self.theta += self.command.angular.z * period
        # Need to publish updated TF
        self.publish()

    def publish(self):
        # Publish base->odom transform
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = sin(self.theta / 2.0)
        t.transform.rotation.w = cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)
        # Publish battery state
        b = BatteryState()
        if self.is_charging:
            b.current = 1.0
        else:
            b.current = -1.0
        self.battery_state_pub.publish(b)

    def action_feedback_callback(self, msg):
        # Force the docking action to run a full recovery loop and then
        # make contact with the dock (based on pose of robot) before
        # we report that the robot is charging
        if msg.feedback.num_retries > 0 and \
                msg.feedback.state == msg.feedback.WAIT_FOR_CHARGE:
            self.is_charging = True

    def nav_execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.x = goal.pose.pose.position.x - 0.05
        self.y = goal.pose.pose.position.y + 0.05
        self.theta = 2.0 * acos(goal.pose.pose.orientation.w)
        self.node.get_logger().info('Navigating to %f %f %f' % (self.x, self.y, self.theta))
        goal_handle.succeed()
        self.publish()

        result = NavigateToPose.Result()
        result.error_code = 0
        result.error_msg = ''
        return result

    def test_docking_server(self):
        # Publish TF for odometry
        self.tf_broadcaster = TransformBroadcaster(self.node)

        # Create a timer to run "control loop" at 20hz
        self.timer = self.node.create_timer(0.05, self.timer_callback)

        # Create action client
        self.dock_action_client = ActionClient(self.node, DockRobot, 'dock_robot')
        self.undock_action_client = ActionClient(self.node, UndockRobot, 'undock_robot')

        # Subscribe to command velocity
        self.node.create_subscription(
            Twist,
            'cmd_vel',
            self.command_velocity_callback,
            10
        )

        # Create publisher for battery state message
        self.battery_state_pub = self.node.create_publisher(
            BatteryState,
            'battery_state',
            10
        )

        # Mock out navigation server (just teleport the robot)
        self.action_server = ActionServer(
            self.node,
            NavigateToPose,
            'navigate_to_pose',
            self.nav_execute_callback)

        # Spin once so that TF is published
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # Test docking action
        self.action_result = []
        self.dock_action_client.wait_for_server(timeout_sec=5.0)
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = 'test_dock'
        future = self.dock_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle.accepted
        result_future_original = self.goal_handle.get_result_async()

        # Run for 2 seconds
        for i in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Send another goal to preempt the first
        future = self.dock_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle.accepted
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.action_result.append(result_future.result())

        rclpy.spin_until_future_complete(self.node, result_future_original)
        self.action_result.append(result_future_original.result())

        # First is aborted due to preemption
        self.assertEqual(self.action_result[0].status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.action_result[0].result.success)

        self.node.get_logger().info('Goal preempted')

        # Run for 0.5 seconds
        for i in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Second is aborted due to preemption during main loop (takes down all actions)
        self.assertEqual(self.action_result[1].status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.action_result[1].result.success)

        # Resend the goal
        self.node.get_logger().info('Sending goal again')
        future = self.dock_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle.accepted
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.action_result.append(result_future.result())

        self.assertEqual(self.action_result[2].status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(self.action_result[2].result.success)
        self.assertEqual(self.action_result[2].result.num_retries, 1)

        # Test undocking action
        self.is_charging = False
        self.undock_action_client.wait_for_server(timeout_sec=5.0)
        goal = UndockRobot.Goal()
        goal.dock_type = 'test_dock_plugin'
        future = self.undock_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle.accepted
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.action_result.append(result_future.result())

        self.assertEqual(self.action_result[3].status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(self.action_result[3].result.success)
