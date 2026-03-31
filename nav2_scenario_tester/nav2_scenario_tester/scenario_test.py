"""
Shared scenario test module for nav2_scenario_tester.

Each scenario only needs a two-line ``test.py``::

    from nav2_scenario_tester.scenario_test import create_test
    generate_test_description, TestParameterizedNavigation = create_test(__file__)

``launch_testing`` discovers ``generate_test_description`` and
``TestParameterizedNavigation`` via the module-level names.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from nav2_scenario_tester import (CostmapMetrics, load_test_suite, NavTestRunner, OdometryMetrics,
                                  PlanMetrics, TrackingMetrics)
import rclpy


def create_test(test_file: str):
    """
    Build a launch description factory and test class for the given scenario.

    Parameters
    ----------
    test_file : str
        The ``__file__`` of the calling ``test.py``.

    Returns
    -------
    tuple
        ``(generate_test_description, TestParameterizedNavigation)`` ready to
        be assigned as module-level names so ``launch_testing`` can discover them.

    """
    scenario_dir = os.path.dirname(os.path.abspath(test_file))
    test_yaml = os.path.join(scenario_dir, 'test_cases.yaml')
    suite = load_test_suite(test_yaml)

    default_bt = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees', 'navigate_w_replanning_only_if_goal_is_updated.xml',
    )
    params_file = suite.params_file or os.path.join(scenario_dir, 'params.yaml')
    bt_xml = suite.bt_xml or default_bt

    def generate_test_description():
        """
        Set up the nav2 stack with loopback simulation.

        If the environment variable ``NAV2_SCENARIO_SKIP_LAUNCH`` is set to a
        non-empty value the loopback stack is **not** launched.  Use this when
        you have already started the environment separately (e.g. via
        ``ros2 launch nav2_scenario_tester loopback_test.launch.py``) and only
        want to run the test assertions against it::

            NAV2_SCENARIO_SKIP_LAUNCH=1 launch_test path/to/scenario/test.py

        The runner will still wait for ``bt_navigator`` to become active before
        executing the first test case.
        """
        if os.environ.get('NAV2_SCENARIO_SKIP_LAUNCH'):
            # Environment is already running — just signal readiness immediately.
            # A keepalive process is required so launch_testing does not exit
            # before the tests complete (it terminates when all processes stop).
            return launch.LaunchDescription([
                ExecuteProcess(cmd=['sleep', 'infinity'], output='screen'),
                TimerAction(
                    period=0.5,
                    actions=[launch_testing.actions.ReadyToTest()],
                ),
            ])

        nav_test_dir = get_package_share_directory('nav2_scenario_tester')

        launch_args = {
            'use_sim_time': 'True',
            'params_file': params_file,
            'rviz': 'true',
        }
        if suite.map_yaml:
            launch_args['map'] = suite.map_yaml

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
                    TrackingMetrics(),
                ],
            )

        @classmethod
        def tearDownClass(cls):
            cls.runner.shutdown()
            cls.runner.destroy_node()
            rclpy.shutdown()

        def test_all_cases(self):
            """Run each test case from the YAML file as a subTest."""
            for i, tc in enumerate(suite.cases, 1):
                with self.subTest(name=tc.name):
                    self.runner.get_logger().info(
                        f'--- [{i}/{len(suite.cases)}] Running: {tc.name} ---'
                    )
                    result = self.runner.run(
                        initial_pose=tc.initial_pose,
                        goal_pose=tc.goal_pose,
                        timeout=tc.timeout,
                        limits=tc.limits,
                        behavior_tree=bt_xml,
                        obstacles=tc.obstacles,
                    )

                    # Log metrics
                    if result.metrics:
                        lines = [f'[{tc.name}] metrics:']
                        for k, v in result.metrics.items():
                            if isinstance(v, tuple):
                                lines.append(f'  {k}: {v[0]:.3f} / {v[1]:.3f}')
                            elif isinstance(v, (int, float)):
                                lines.append(f'  {k}: {v:.3f}')
                        self.runner.get_logger().info('\n'.join(lines))

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
                            # Tuples are (min, max); use max for limit checks
                            if isinstance(value, tuple):
                                value = value[1]
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

    return generate_test_description, TestParameterizedNavigation
