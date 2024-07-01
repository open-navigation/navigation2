#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration


# Shelf positions for picking
shelf_positions = {
    'shelf_A': [8.21, 1.3, 1.57],
    'shelf_B': [11.6, 1.3, -1.57],
    'shelf_C': [12.6, 4.0, -1.57],
    'shelf_D':  [19.0, 4.0, 1.57],
}

# Shipping destination for picked products
shipping_destinations = {
    'frieght_bay_1': [-6.0, 5.0, 3.14],
    'frieght_bay_2': [-6.0, 0.0, 3.14],
    'frieght_bay_3': [-6.0, -5.0, 3.14],
}

"""
Basic item picking demo. In this demonstration, the expectation
is that there is a person at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with some kind of button for 'got item, robot go do next task').
"""


def main():
    # Recieved virtual request for picking item at Shelf A and bring to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ('shelf_A') and shipping destination ('frieght_bay_3')
    ####################
    request_item_location = 'shelf_A'
    request_destination = 'frieght_bay_3'
    ####################

    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    # Simplification of angle handling for demonstration purposes
    if shelf_positions[request_item_location][2] > 0:
        shelf_item_pose.pose.orientation.z = 0.707
        shelf_item_pose.pose.orientation.w = 0.707
    else:
        shelf_item_pose.pose.orientation.z = -0.707
        shelf_item_pose.pose.orientation.w = 0.707
    print(f'Received request for item picking at {request_item_location}.')
    navigator.goToPose(shelf_item_pose)

    # Do something during our route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Simply print information for workers on the robot's ETA for the demonstation
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival at '
                + request_item_location
                + ' for worker: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(
            'Got product from '
            + request_item_location
            + '! Bringing product to shipping destination ('
            + request_destination
            + ')...'
        )
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = shipping_destinations[
            request_destination
        ][0]
        shipping_destination.pose.position.y = shipping_destinations[
            request_destination
        ][1]
        shipping_destination.pose.orientation.z = 1.0
        shipping_destination.pose.orientation.w = 0.0
        navigator.goToPose(shipping_destination)

    elif result == TaskResult.CANCELED:
        print(
            f'Task at {request_item_location} was canceled. Returning to staging point...'
        )
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        (error_code, error_msg) = navigator.getLastActionError()
        print(f'Task at {request_item_location} failed!:'
              f'{error_code}:{error_msg}')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()
