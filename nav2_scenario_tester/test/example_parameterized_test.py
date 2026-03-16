# Copyright (c) 2026, Dexory (Tony Najjar)
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
Example parameterized test using YAML-defined test cases.

Loads test cases from a YAML file and runs each as a subTest, collecting
metrics and checking limits. This pattern scales well for large test suites.

Usage from CMakeLists.txt:
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/example_parameterized_test.py TIMEOUT 180)
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from nav2_scenario_tester import (CostmapMetrics, load_test_suite, NavTestRunner, OdometryMetrics,
                                  PlanMetrics)
import rclpy

TEST_YAML = os.path.join(os.path.dirname(__file__), 'warehouse_test_cases.yaml')
TEST_SUITE = load_test_suite(TEST_YAML)


def generate_test_description():
    """Set up the nav2 stack with loopback simulation."""
    nav_test_dir = get_package_share_directory('nav2_scenario_tester')

    launch_args = {
        'use_sim_time': 'True',
        'params_file': os.path.join(nav_test_dir, 'config', 'default_test_params.yaml'),
        'rviz': 'false',
    }
    if TEST_SUITE.map_yaml:
        launch_args['map'] = TEST_SUITE.map_yaml

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_test_dir, 'launch', 'loopback_test.launch.py')
        ),
        launch_arguments=launch_args.items(),
    )

    return launch.LaunchDescription([
        nav_stack,
        TimerAction(
            period=2.0,
            actions=[launch_testing.actions.ReadyToTest()],
        ),
    ])


class TestParameterizedNavigation(unittest.TestCase):
    """Run all YAML-defined test cases with metrics and limit checking."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.runner = NavTestRunner(
            metrics_collectors=[
                OdometryMetrics(),
                PlanMetrics(),
                CostmapMetrics(),
            ],
        )

    @classmethod
    def tearDownClass(cls):
        cls.runner.shutdown()
        cls.runner.destroy_node()
        rclpy.shutdown()

    def test_all_cases(self):
        """Run each test case from the YAML file as a subTest."""
        for tc in TEST_SUITE.cases:
            with self.subTest(name=tc.name):
                result = self.runner.run(
                    initial_pose=tc.initial_pose,
                    goal_pose=tc.goal_pose,
                    timeout=tc.timeout,
                    limits=tc.limits,
                )

                # Log metrics
                if result.metrics:
                    self.runner.get_logger().info(
                        f'[{tc.name}] metrics: ' + ', '.join(
                            f'{k}={v:.3f}' for k, v in result.metrics.items()
                            if isinstance(v, (int, float))
                        )
                    )

                # Assert navigation succeeded
                self.assertTrue(
                    result.success,
                    f'{tc.name} failed: error_code={result.error_code}, '
                    f'error_msg={result.error_msg}',
                )

                # Assert metric limits
                if tc.limits and result.metrics:
                    for metric, bounds in tc.limits.items():
                        value = result.metrics.get(metric)
                        if value is None:
                            continue
                        lo, hi = bounds
                        if lo is not None:
                            self.assertGreaterEqual(
                                value, lo,
                                f'{tc.name}: {metric}={value:.3f} below min {lo}',
                            )
                        if hi is not None:
                            self.assertLessEqual(
                                value, hi,
                                f'{tc.name}: {metric}={value:.3f} exceeds max {hi}',
                            )
