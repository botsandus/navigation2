# Copyright (c) 2026 nav2_nav_tester contributors
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
``launch_testing``-based test launch file.

Brings up a navigation backend (loopback or gazebo) and exposes a
``ReadyToTest`` gate so that ``colcon test`` or ``launch_test`` can drive
scenario execution through standard unittest/pytest assertions.

Usage::

    launch_test nav_test.launch.py

Environment variables:
    NAV_TEST_BACKEND   - "loopback" or "gazebo" (default: loopback)
    NAV_TEST_PARAMS    - override params file
    NAV_TEST_MAP       - override map YAML
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
from nav2_nav_tester.discovery import discover_scenarios
from nav2_nav_tester.runner import run_scenario, TestStatus
from nav2_nav_tester.scenario import load_scenario
from nav2_simple_commander.robot_navigator import BasicNavigator
import pytest
import rclpy


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    pkg_dir = get_package_share_directory('nav2_nav_tester')

    backend = os.environ.get('NAV_TEST_BACKEND', 'loopback')
    params_override = os.environ.get('NAV_TEST_PARAMS', '')
    map_override = os.environ.get('NAV_TEST_MAP', '')

    if not params_override:
        if backend == 'loopback':
            params_override = os.path.join(
                pkg_dir, 'config', 'loopback_params.yaml',
            )
        else:
            sys_tests_dir = get_package_share_directory('nav2_system_tests')
            params_override = os.path.join(
                sys_tests_dir, 'nav2_system_params.yaml',
            )

    if backend == 'loopback':
        from nav2_nav_tester.backends.loopback import generate_nav_actions
        nav_actions = generate_nav_actions(
            params_file=params_override,
            map_yaml=map_override,
        )
    elif backend == 'gazebo':
        from nav2_nav_tester.backends.gazebo import generate_nav_actions
        nav_actions = generate_nav_actions(
            params_file=params_override,
            map_yaml=map_override,
        )
    else:
        raise ValueError(
            f"Unknown backend '{backend}'. Use 'loopback' or 'gazebo'."
        )

    return launch.LaunchDescription(
        nav_actions + [launch_testing.actions.ReadyToTest()],
    )


class TestNavScenarios(unittest.TestCase):
    """Run every discovered scenario as an individual test case."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.navigator = BasicNavigator()
        cls.navigator.waitUntilNav2Active(
            navigator='bt_navigator', localizer='robot_localization',
        )
        cls.scenarios = [
            load_scenario(f) for f in discover_scenarios()
        ]

    @classmethod
    def tearDownClass(cls):
        cls.navigator.lifecycleShutdown()
        rclpy.shutdown()

    def test_all_scenarios(self):
        """Execute every discovered scenario and assert success."""
        failures = []
        for sc in self.scenarios:
            with self.subTest(scenario=sc.name):
                result = run_scenario(self.navigator, sc)
                if result.status != TestStatus.SUCCEEDED:
                    failures.append(
                        f'{sc.name}: {result.status.name} — {result.message}'
                    )
                self.assertEqual(
                    result.status,
                    TestStatus.SUCCEEDED,
                    f'{sc.name} failed: {result.message}',
                )
