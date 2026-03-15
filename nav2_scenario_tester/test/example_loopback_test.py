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
Example loopback integration test using nav2_scenario_tester.

Demonstrates a simple single-goal navigation test with metrics collection.

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
from nav2_scenario_tester import NavTestRunner, OdometryMetrics, PlanMetrics
from nav2_scenario_tester.test_runner import make_pose
import rclpy


def generate_test_description():
    """Set up the nav2 stack with loopback simulation."""
    nav_test_dir = get_package_share_directory('nav2_scenario_tester')

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
        TimerAction(
            period=2.0,
            actions=[launch_testing.actions.ReadyToTest()],
        ),
    ])


class TestLoopbackNavigation(unittest.TestCase):
    """Simple loopback navigation test — single goal with metrics."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.runner = NavTestRunner(
            metrics_collectors=[OdometryMetrics(), PlanMetrics()],
        )

    @classmethod
    def tearDownClass(cls):
        cls.runner.shutdown()
        cls.runner.destroy_node()
        rclpy.shutdown()

    def test_short_path(self):
        """Navigate a short straight path and verify success."""
        result = self.runner.run(
            initial_pose=make_pose(-2.0, -0.5),
            goal_pose=make_pose(0.0, -0.5),
            timeout=60.0,
        )
        self.assertTrue(
            result.success,
            f'Navigation failed: error_code={result.error_code}, '
            f'error_msg={result.error_msg}',
        )
        if result.metrics:
            self.runner.get_logger().info(
                'Metrics: ' + ', '.join(
                    f'{k}={v:.3f}' for k, v in result.metrics.items()
                )
            )

    def test_longer_path(self):
        """Navigate a longer path and check distance limit."""
        result = self.runner.run(
            initial_pose=make_pose(-2.0, -0.5),
            goal_pose=make_pose(0.5, 1.0),
            timeout=90.0,
            limits={'distance_travelled': [None, 6.0]},
        )
        self.assertTrue(
            result.success,
            f'Navigation failed: error_code={result.error_code}, '
            f'error_msg={result.error_msg}',
        )
