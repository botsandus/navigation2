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

"""
Example loopback integration test using nav2_navigation_test.

Demonstrates Option A: full launch_testing with NavTestRunner for
custom assertions and reporting.

This test:
  1. Launches the nav2 stack with the loopback simulator
  2. Uses NavTestRunner to send a NavigateToPose goal
  3. Asserts the robot reached the goal within tolerance

Run manually:
  launch_test test/example_loopback_test.py

Register in CMakeLists.txt:
  add_launch_test(test/example_loopback_test.py TIMEOUT 120)
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from nav2_navigation_test import NavTestRunner
from nav2_navigation_test.test_runner import make_pose
import rclpy


def generate_test_description():
    """Set up the nav2 stack with loopback simulation."""
    nav_test_dir = get_package_share_directory('nav2_navigation_test')

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_test_dir, 'launch', 'navigation_test.launch.py')
        ),
        launch_arguments={
            'sim_type': 'loopback',
            'use_sim_time': 'True',
        }.items(),
    )

    return launch.LaunchDescription([
        nav_stack,
        # Allow time for all nodes to start before running tests
        TimerAction(
            period=2.0,
            actions=[launch_testing.actions.ReadyToTest()],
        ),
    ])


class TestLoopbackNavigation(unittest.TestCase):
    """Integration test for navigation with loopback simulator."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.runner = NavTestRunner()

    @classmethod
    def tearDownClass(cls):
        cls.runner.shutdown()
        cls.runner.destroy_node()
        rclpy.shutdown()

    def test_navigate_to_pose(self):
        """Test that the robot can navigate from start to goal."""
        result = self.runner.run(
            initial_pose=make_pose(-2.0, -0.5),
            goal_pose=make_pose(0.0, 2.0),
            timeout=90.0,
        )
        self.assertTrue(
            result.success,
            f'Navigation failed: '
            f'error_code={result.error_code}, error_msg={result.error_msg}',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Checks that all processes exited cleanly."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
