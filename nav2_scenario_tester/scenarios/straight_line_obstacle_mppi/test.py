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

TEST_YAML = os.path.join(os.path.dirname(__file__), 'test_cases.yaml')
TEST_SUITE = load_test_suite(TEST_YAML)
BT_XML = os.path.join(
    get_package_share_directory('nav2_bt_navigator'),
    'behavior_trees', 'navigate_to_pose_simple.xml',
)


def generate_test_description():
    """Set up the nav2 stack with loopback simulation."""
    nav_test_dir = get_package_share_directory('nav2_scenario_tester')

    launch_args = {
        'use_sim_time': 'True',
        'params_file': os.path.join(os.path.dirname(__file__), 'params.yaml'),
        'rviz': 'true',
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
                    behavior_tree=BT_XML,
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
