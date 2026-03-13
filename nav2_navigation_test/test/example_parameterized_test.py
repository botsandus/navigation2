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
Example: Parameterized navigation test with different configurations.

Demonstrates how to test the same navigation scenario with different
planner/controller tunings using the CMake helper (Option B).

To register multiple tests from a downstream package's CMakeLists.txt:

  find_package(nav2_navigation_test REQUIRED)

  # Test with NavfnPlanner + DWB (default config)
  nav2_navigation_add_test(nav_default_loopback
    SIM_TYPE loopback
    START_POSE "-2.0;-0.5;0.0"
    GOAL_POSE "0.0;2.0;0.0"
    TIMEOUT 120
  )

  # Test with custom MPPI config
  nav2_navigation_add_test(nav_mppi_loopback
    SIM_TYPE loopback
    PARAMS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/config/mppi_params.yaml
    START_POSE "-2.0;-0.5;0.0"
    GOAL_POSE "0.0;2.0;0.0"
    TIMEOUT 120
  )

  # Test with a custom map
  nav2_navigation_add_test(nav_custom_map
    SIM_TYPE loopback
    PARAMS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/config/my_params.yaml
    MAP ${CMAKE_CURRENT_SOURCE_DIR}/maps/warehouse.yaml
    START_POSE "1.0;1.0;0.0"
    GOAL_POSE "5.0;3.0;1.57"
    TIMEOUT 180
  )

This file is a standalone launch_testing example that runs two navigation
scenarios sequentially within the same test to compare results.
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

# Define test scenarios as (name, start_pose, goal_pose) tuples
TEST_SCENARIOS = [
    ('short_forward', make_pose(-2.0, -0.5), make_pose(0.0, 2.0)),
    ('diagonal', make_pose(-2.0, -0.5), make_pose(2.0, 0.5)),
]


def generate_test_description():
    """Set up nav2 stack with loopback simulation."""
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
        TimerAction(
            period=2.0,
            actions=[launch_testing.actions.ReadyToTest()],
        ),
    ])


class TestParameterizedNavigation(unittest.TestCase):
    """Run multiple navigation scenarios on the same persistent stack."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.runner = NavTestRunner()

    @classmethod
    def tearDownClass(cls):
        cls.runner.shutdown()
        cls.runner.destroy_node()
        rclpy.shutdown()

    def test_all_scenarios(self):
        """Navigate through all defined scenarios, teleporting between each."""
        for name, start, goal in TEST_SCENARIOS:
            with self.subTest(scenario=name):
                result = self.runner.run(
                    initial_pose=start,
                    goal_pose=goal,
                    timeout=90.0,
                )
                self.assertTrue(
                    result.success,
                    f'Scenario "{name}" failed: '
                    f'error_code={result.error_code}',
                )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
