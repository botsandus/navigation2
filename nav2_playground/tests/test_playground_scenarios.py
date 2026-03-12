# Copyright (c) 2026 nav2_playground contributors
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

"""
Example navigation test scenarios for the playground.

Launch the playground first, then run:
    ros2 run nav2_playground playground_test_runner

Or use these test cases in your own test scripts.
"""

import math
import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.launch_description_sources
import launch_testing
import launch_testing.actions
import launch_testing.markers
from nav2_nav_tester.runner import run_scenario, TestStatus
from nav2_nav_tester.scenario import Scenario
from nav2_simple_commander.robot_navigator import BasicNavigator
import pytest
import rclpy


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    playground_dir = get_package_share_directory('nav2_playground')
    bringup_dir = get_package_share_directory('nav2_bringup')

    playground_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(playground_dir, 'launch', 'playground.launch.py')
        ),
        launch_arguments={
            'map': os.path.join(bringup_dir, 'maps', 'tb3_sandbox.yaml'),
            'use_rviz': 'False',
            'autostart': 'true',
        }.items(),
    )

    return launch.LaunchDescription([
        playground_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestPlaygroundNavigation(unittest.TestCase):
    """Integration tests that require the playground to be running."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.navigator = BasicNavigator()
        # Skip localizer wait — loopback sim provides TF directly
        cls.navigator.waitUntilNav2Active(
            navigator='bt_navigator', localizer='robot_localization'
        )

    @classmethod
    def tearDownClass(cls):
        cls.navigator.lifecycleShutdown()
        rclpy.shutdown()

    def test_straight_line(self):
        """Robot navigates in a straight line forward."""
        sc = Scenario(
            name='straight_line_forward',
            description='Straight line forward',
            start_x=-2.0, start_y=-0.5, start_yaw=0.0,
            goal_x=0.0, goal_y=-0.5, goal_yaw=0.0,
            timeout_sec=30.0,
        )
        result = run_scenario(self.navigator, sc)
        self.assertEqual(result.status, TestStatus.SUCCEEDED,
                         f'Navigation failed: {result.message}')

    def test_diagonal_move(self):
        """Robot navigates diagonally with a heading change."""
        sc = Scenario(
            name='diagonal_move',
            description='Diagonal with heading change',
            start_x=-2.0, start_y=-0.5, start_yaw=0.0,
            goal_x=0.0, goal_y=1.0, goal_yaw=math.pi / 2,
            timeout_sec=45.0,
        )
        result = run_scenario(self.navigator, sc)
        self.assertEqual(result.status, TestStatus.SUCCEEDED,
                         f'Navigation failed: {result.message}')

    def test_reverse_direction(self):
        """Robot navigates backward to a pose behind its start."""
        sc = Scenario(
            name='reverse_direction',
            description='Navigate backward',
            start_x=0.0, start_y=-0.5, start_yaw=0.0,
            goal_x=-2.0, goal_y=-0.5, goal_yaw=math.pi,
            timeout_sec=45.0,
        )
        result = run_scenario(self.navigator, sc)
        self.assertEqual(result.status, TestStatus.SUCCEEDED,
                         f'Navigation failed: {result.message}')
