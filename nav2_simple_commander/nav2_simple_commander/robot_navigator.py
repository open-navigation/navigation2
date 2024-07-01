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


from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import AssistedTeleop, BackUp, DriveOnHeading, Spin
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import (
    DockRobot,
    FollowGPSWaypoints,
    FollowPath,
    FollowWaypoints,
    NavigateThroughPoses,
    NavigateToPose,
    UndockRobot,
)
from nav2_msgs.action import SmoothPath
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class BasicNavigator(Node):

    def __init__(self, node_name='basic_navigator', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.last_action_error_code = 0
        self.last_action_error_msg = ''

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses'
        )
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints'
        )
        self.follow_gps_waypoints_client = ActionClient(
            self, FollowGPSWaypoints, 'follow_gps_waypoints'
        )
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.compute_path_to_pose_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose'
        )
        self.compute_path_through_poses_client = ActionClient(
            self, ComputePathThroughPoses, 'compute_path_through_poses'
        )
        self.smoother_client = ActionClient(self, SmoothPath, 'smooth_path')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.backup_client = ActionClient(self, BackUp, 'backup')
        self.drive_on_heading_client = ActionClient(
            self, DriveOnHeading, 'drive_on_heading'
        )
        self.assisted_teleop_client = ActionClient(
            self, AssistedTeleop, 'assisted_teleop'
        )
        self.docking_client = ActionClient(self, DockRobot, 'dock_robot')
        self.undocking_client = ActionClient(self, UndockRobot, 'undock_robot')
        self.localization_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._amclPoseCallback,
            amcl_pose_qos,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        self.change_maps_srv = self.create_client(LoadMap, 'map_server/load_map')
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap'
        )
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap'
        )
        self.get_costmap_global_srv = self.create_client(
            GetCostmap, 'global_costmap/get_costmap'
        )
        self.get_costmap_local_srv = self.create_client(
            GetCostmap, 'local_costmap/get_costmap'
        )

    def destroyNode(self):
        self.destroy_node()

    def destroy_node(self):
        self.nav_through_poses_client.destroy()
        self.nav_to_pose_client.destroy()
        self.follow_waypoints_client.destroy()
        self.follow_path_client.destroy()
        self.compute_path_to_pose_client.destroy()
        self.compute_path_through_poses_client.destroy()
        self.smoother_client.destroy()
        self.spin_client.destroy()
        self.backup_client.destroy()
        self.drive_on_heading_client.destroy()
        super().destroy_node()

    def setInitialPose(self, initial_pose):
        """Set the initial pose to the localization system."""
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goThroughPoses(self, poses, behavior_tree=''):
        """Send a `NavThroughPoses` action request."""
        self.clearLastActionError()
        self.debug("Waiting for 'NavigateThroughPoses' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateThroughPoses' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        goal_msg.behavior_tree = behavior_tree

        self.info(f'Navigating with {len(goal_msg.poses)} goals....')
        send_goal_future = self.nav_through_poses_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f'Goal with {len(poses)} poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.clearLastActionError()
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info(
            'Navigating to goal: '
            + str(pose.pose.position.x)
            + ' '
            + str(pose.pose.position.y)
            + '...'
        )
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(
                'Goal to '
                + str(pose.pose.position.x)
                + ' '
                + str(pose.pose.position.y)
                + ' was rejected!'
            )
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followWaypoints(self, poses):
        """Send a `FollowWaypoints` action request."""
        self.clearLastActionError()
        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.info(f'Following {len(goal_msg.poses)} goals....')
        send_goal_future = self.follow_waypoints_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f'Following {len(poses)} waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followGpsWaypoints(self, gps_poses):
        """Send a `FollowGPSWaypoints` action request."""
        self.clearLastActionError()
        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_gps_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowGPSWaypoints.Goal()
        goal_msg.gps_poses = gps_poses

        self.info(f'Following {len(goal_msg.gps_poses)} gps goals....')
        send_goal_future = self.follow_gps_waypoints_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(
                f'Following {len(gps_poses)} gps waypoints request was rejected!'
            )
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def spin(self, spin_dist=1.57, time_allowance=10):
        self.clearLastActionError()
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def backup(self, backup_dist=0.15, backup_speed=0.025, time_allowance=10):
        self.clearLastActionError()
        self.debug("Waiting for 'Backup' action server")
        while not self.backup_client.wait_for_server(timeout_sec=1.0):
            self.info("'Backup' action server not available, waiting...")
        goal_msg = BackUp.Goal()
        goal_msg.target = Point(x=float(backup_dist))
        goal_msg.speed = backup_speed
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Backing up {goal_msg.target.x} m at {goal_msg.speed} m/s....')
        send_goal_future = self.backup_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Backup request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def driveOnHeading(self, dist=0.15, speed=0.025, time_allowance=10):
        self.clearLastActionError()
        self.debug("Waiting for 'DriveOnHeading' action server")
        while not self.backup_client.wait_for_server(timeout_sec=1.0):
            self.info("'DriveOnHeading' action server not available, waiting...")
        goal_msg = DriveOnHeading.Goal()
        goal_msg.target = Point(x=float(dist))
        goal_msg.speed = speed
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Drive {goal_msg.target.x} m on heading at {goal_msg.speed} m/s....')
        send_goal_future = self.drive_on_heading_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Drive On Heading request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def assistedTeleop(self, time_allowance=30):
        self.clearLastActionError()
        self.debug("Wainting for 'assisted_teleop' action server")
        while not self.assisted_teleop_client.wait_for_server(timeout_sec=1.0):
            self.info("'assisted_teleop' action server not available, waiting...")
        goal_msg = AssistedTeleop.Goal()
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info("Running 'assisted_teleop'....")
        send_goal_future = self.assisted_teleop_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Assisted Teleop request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followPath(self, path, controller_id='', goal_checker_id=''):
        self.clearLastActionError()
        """Send a `FollowPath` action request."""
        self.debug("Waiting for 'FollowPath' action server")
        while not self.follow_path_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowPath' action server not available, waiting...")

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = controller_id
        goal_msg.goal_checker_id = goal_checker_id

        self.info('Executing path...')
        send_goal_future = self.follow_path_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Follow path was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def dockRobotByPose(self, dock_pose, dock_type, nav_to_dock=True):
        self.clearLastActionError()
        """Send a `DockRobot` action request."""
        self.info("Waiting for 'DockRobot' action server")
        while not self.docking_client.wait_for_server(timeout_sec=1.0):
            self.info('"DockRobot" action server not available, waiting...')

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = False
        goal_msg.dock_pose = dock_pose
        goal_msg.dock_type = dock_type
        goal_msg.navigate_to_staging_pose = nav_to_dock  # if want to navigate before staging

        self.info('Docking at pose: ' + str(dock_pose) + '...')
        send_goal_future = self.docking_client.send_goal_async(goal_msg,
                                                               self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.SetLastError(DockRobot.UNKNOWN, 'Docking request was rejected')
            self.info('Docking request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def dockRobotByID(self, dock_id, nav_to_dock=True):
        """Send a `DockRobot` action request."""
        self.clearLastActionError()
        self.info("Waiting for 'DockRobot' action server")
        while not self.docking_client.wait_for_server(timeout_sec=1.0):
            self.info('"DockRobot" action server not available, waiting...')

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = dock_id
        goal_msg.navigate_to_staging_pose = nav_to_dock  # if want to navigate before staging

        self.info('Docking at dock ID: ' + str(dock_id) + '...')
        send_goal_future = self.docking_client.send_goal_async(goal_msg,
                                                               self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.SetLastError(DockRobot.UNKNOWN, 'Docking request was rejected')
            self.info('Docking request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def undockRobot(self, dock_type=''):
        """Send a `UndockRobot` action request."""
        self.clearLastActionError()
        self.info("Waiting for 'UndockRobot' action server")
        while not self.undocking_client.wait_for_server(timeout_sec=1.0):
            self.info('"UndockRobot" action server not available, waiting...')

        goal_msg = UndockRobot.Goal()
        goal_msg.dock_type = dock_type

        self.info('Undocking from dock of type: ' + str(dock_type) + '...')
        send_goal_future = self.undocking_client.send_goal_async(goal_msg,
                                                                 self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.SetLastError(UndockRobot.UNKNOWN, 'Undocking request was rejected')
            self.info('Undocking request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.clearLastActionError()
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self) -> bool:
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                result = self.result_future.result().result
                self.setLastActionError(result.error_code, result.error_msg)
                self.debug('Task with failed with status code:'
                           f'{self.status}:{result.error_code}:{result.error_msg}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def clearLastActionError(self):
        self.last_action_error_code = 0
        self.last_action_error_msg = ''

    def setLastActionError(self, error_code, error_msg):
        self.last_action_error_code = error_code
        self.last_action_error_msg = error_msg

    def getLastActionError(self) -> tuple[int, str]:
        return (self.last_action_error_code, self.last_action_error_msg)

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running."""
        if localizer != 'robot_localization':  # non-lifecycle node
            self._waitForNodeToActivate(localizer)
        if localizer == 'amcl':
            self._waitForInitialPose()
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def _getPathImpl(
            self, start, goal, planner_id='', use_start=False
            ) -> ComputePathToPose.Result:
        """
        Send a `ComputePathToPose` action request.

        Internal implementation to get the full result, not just the path.
        """
        self.debug("Waiting for 'ComputePathToPose' action server")
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = planner_id
        goal_msg.use_start = use_start

        self.info('Getting path...')
        send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            self.status = GoalStatus.UNKNOWN
            result = ComputePathToPose.Result()
            result.error_code = ComputePathToPose.NONE
            result.error_msg = 'Get path was rejected'
            return result

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status

        return self.result_future.result().result

    def getPath(self, start, goal, planner_id='', use_start=False):
        """Send a `ComputePathToPose` action request."""
        self.clearLastActionError()
        rtn = self._getPathImpl(start, goal, planner_id, use_start)

        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return rtn.path
        else:
            self.setLastActionError(rtn.error_code, rtn.error_msg)
            self.warn('Getting path failed with status code:'
                      f'{self.status}:{rtn.error_code}:{rtn.error_msg}')
            return None

    def _getPathThroughPosesImpl(
        self, start, goals, planner_id='', use_start=False
    ) -> ComputePathThroughPoses.Result:
        """
        Send a `ComputePathThroughPoses` action request.

        Internal implementation to get the full result, not just the path.
        """
        self.debug("Waiting for 'ComputePathThroughPoses' action server")
        while not self.compute_path_through_poses_client.wait_for_server(
            timeout_sec=1.0
        ):
            self.info(
                "'ComputePathThroughPoses' action server not available, waiting..."
            )

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.start = start
        goal_msg.goals = goals
        goal_msg.planner_id = planner_id
        goal_msg.use_start = use_start

        self.info('Getting path...')
        send_goal_future = self.compute_path_through_poses_client.send_goal_async(
            goal_msg
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            result = ComputePathThroughPoses.Result()
            result.error_code = ComputePathThroughPoses.UNKNOWN
            result.error_msg = 'Get path was rejected!'
            return result

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status

        return self.result_future.result().result

    def getPathThroughPoses(self, start, goals, planner_id='', use_start=False):
        """Send a `ComputePathThroughPoses` action request."""
        self.clearLastActionError()
        rtn = self._getPathThroughPosesImpl(start, goals, planner_id, use_start)

        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return rtn.path
        else:
            self.setLastActionError(rtn.error_code, rtn.error_msg)
            self.warn('Getting path failed with status code:'
                      f'{self.status}:{rtn.error_code}:{rtn.error_msg}')
            return None

    def _smoothPathImpl(
        self, path, smoother_id='', max_duration=2.0, check_for_collision=False
    ) -> SmoothPath.Result:
        """
        Send a `SmoothPath` action request.

        Internal implementation to get the full result, not just the path.
        """
        self.debug("Waiting for 'SmoothPath' action server")
        while not self.smoother_client.wait_for_server(timeout_sec=1.0):
            self.info("'SmoothPath' action server not available, waiting...")

        goal_msg = SmoothPath.Goal()
        goal_msg.path = path
        goal_msg.max_smoothing_duration = rclpyDuration(seconds=max_duration).to_msg()
        goal_msg.smoother_id = smoother_id
        goal_msg.check_for_collisions = check_for_collision

        self.info('Smoothing path...')
        send_goal_future = self.smoother_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Smooth path was rejected!')
            result = SmoothPath.Result()
            result.error_code = SmoothPath.UNKNOWN
            result.error_msg = 'Smooth path was rejected'
            return result

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status

        return self.result_future.result().result

    def smoothPath(
        self, path, smoother_id='', max_duration=2.0, check_for_collision=False
    ):
        """Send a `SmoothPath` action request."""
        self.clearLastActionError()
        rtn = self._smoothPathImpl(path, smoother_id, max_duration, check_for_collision)

        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return rtn.path
        else:
            self.setLastActionError(rtn.error_code, rtn.error_msg)
            self.warn('Getting path failed with status code:'
                      f'{self.status}:{rtn.error_code}:{rtn.error_msg}')
            return None

    def changeMap(self, map_filepath) -> bool:
        """Change the current static map in the map server."""
        while not self.change_maps_srv.wait_for_service(timeout_sec=1.0):
            self.info('change map service not available, waiting...')
        req = LoadMap.Request()
        req.map_url = map_filepath
        future = self.change_maps_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result().result
        if result != LoadMap.Response().RESULT_SUCCESS:
            if result == LoadMap.RESULT_MAP_DOES_NOT_EXIST:
                reason = 'Map does not exist'
            elif result == LoadMap.INVALID_MAP_DATA:
                reason = 'Invalid map data'
            elif result == LoadMap.INVALID_MAP_METADATA:
                reason = 'Invalid map metadata'
            elif result == LoadMap.UNDEFINED_FAILURE:
                reason = 'Undefined failure'
            else:
                reason = 'Unknown'
            self.setLastActionError(result, reason)
            self.error(f'Change map request failed:{reason}!')
            return False
        else:
            self.info('Change map request was successful!')
            return True

    def clearAllCostmaps(self):
        """Clear all costmaps."""
        self.clearLocalCostmap()
        self.clearGlobalCostmap()
        return

    def clearLocalCostmap(self):
        """Clear local costmap."""
        while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear local costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def clearGlobalCostmap(self):
        """Clear global costmap."""
        while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear global costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def getGlobalCostmap(self):
        """Get the global costmap."""
        while not self.get_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get global costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def getLocalCostmap(self):
        """Get the local costmap."""
        while not self.get_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get local costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def lifecycleStartup(self):
        """Startup nav2 lifecycle system."""
        self.info('Starting up lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Starting up {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().STARTUP
                future = mgr_client.call_async(req)

                # starting up requires a full map->odom->base_link TF tree
                # so if we're not successful, try forwarding the initial pose
                while True:
                    rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
                    if not future:
                        self._waitForInitialPose()
                    else:
                        break
        self.info('Nav2 is ready for use!')
        return

    def lifecycleShutdown(self):
        """Shutdown nav2 lifecycle system."""
        self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Shutting down {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
