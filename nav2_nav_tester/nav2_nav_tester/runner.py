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
Core test runner.

Load scenarios, send navigation goals via BasicNavigator, report results.

Can be used:
  - Via ``launch_testing`` / ``colcon test`` (preferred)
  - As a console script (``ros2 run nav2_nav_tester nav_test_runner``)
  - Imported from other packages (playground, system_tests, private repos)
"""

import argparse
from dataclasses import dataclass
from enum import auto, Enum
import math
from pathlib import Path
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_nav_tester.discovery import discover_scenarios
from nav2_nav_tester.scenario import load_scenario, Scenario
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


class TestStatus(Enum):
    SUCCEEDED = auto()
    FAILED = auto()
    TIMED_OUT = auto()
    REJECTED = auto()


@dataclass
class TestResult:
    """Result of a single navigation test."""

    scenario: Scenario
    status: TestStatus
    elapsed_sec: float = 0.0
    message: str = ''


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------

def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _make_pose(x: float, y: float, yaw: float,
               navigator: BasicNavigator) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = _yaw_to_quaternion(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


# ---------------------------------------------------------------------------
# Single-scenario execution
# ---------------------------------------------------------------------------

def run_scenario(navigator: BasicNavigator, scenario: Scenario) -> TestResult:
    """Execute a single navigation scenario. Returns the result."""
    # Set initial pose
    start_pose = _make_pose(
        scenario.start_x, scenario.start_y, scenario.start_yaw, navigator,
    )
    navigator.setInitialPose(start_pose)
    time.sleep(2.0)  # wait for TF propagation

    # Send navigation goal
    goal_pose = _make_pose(
        scenario.goal_x, scenario.goal_y, scenario.goal_yaw, navigator,
    )
    bt = scenario.behavior_tree or ''
    task = navigator.goToPose(goal_pose, behavior_tree=bt)

    if task is None:
        return TestResult(
            scenario=scenario,
            status=TestStatus.REJECTED,
            message='Goal was rejected by the navigator',
        )

    # Poll for completion
    t_start = time.monotonic()
    while not navigator.isTaskComplete(task=task):
        elapsed = time.monotonic() - t_start
        if elapsed > scenario.timeout_sec:
            navigator.cancelTask()
            return TestResult(
                scenario=scenario,
                status=TestStatus.TIMED_OUT,
                elapsed_sec=elapsed,
                message=f'Timed out after {elapsed:.1f}s '
                        f'(limit: {scenario.timeout_sec}s)',
            )

    elapsed = time.monotonic() - t_start
    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        return TestResult(
            scenario=scenario, status=TestStatus.SUCCEEDED,
            elapsed_sec=elapsed,
        )
    elif result == TaskResult.CANCELED:
        return TestResult(
            scenario=scenario, status=TestStatus.FAILED,
            elapsed_sec=elapsed, message='Navigation was canceled',
        )
    else:
        return TestResult(
            scenario=scenario, status=TestStatus.FAILED,
            elapsed_sec=elapsed,
            message=f'Navigation failed with result: {result}',
        )


# ---------------------------------------------------------------------------
# Multi-scenario execution
# ---------------------------------------------------------------------------

def run_scenarios(
    navigator: BasicNavigator,
    scenarios: list[Scenario],
) -> list[TestResult]:
    """Run all scenarios sequentially. Returns list of results."""
    results: list[TestResult] = []
    for i, sc in enumerate(scenarios, 1):
        print(f'\n[{i}/{len(scenarios)}] Running: {sc.name}')
        print(f'  Start: ({sc.start_x}, {sc.start_y}, yaw={sc.start_yaw:.2f})')
        print(f'  Goal:  ({sc.goal_x}, {sc.goal_y}, yaw={sc.goal_yaw:.2f})')
        print(f'  Timeout: {sc.timeout_sec}s')

        result = run_scenario(navigator, sc)
        results.append(result)

        status_str = result.status.name
        if result.status == TestStatus.SUCCEEDED:
            print(f'  Result: \033[32m{status_str}\033[0m '
                  f'({result.elapsed_sec:.1f}s)')
        else:
            print(f'  Result: \033[31m{status_str}\033[0m '
                  f'({result.elapsed_sec:.1f}s)')
            if result.message:
                print(f'  Message: {result.message}')

    # Summary
    passed = sum(1 for r in results if r.status == TestStatus.SUCCEEDED)
    total = len(results)
    print(f'\n{"=" * 60}')
    print(f'Results: {passed}/{total} passed')
    print(f'{"=" * 60}')
    for r in results:
        mark = ('\033[32mPASS\033[0m' if r.status == TestStatus.SUCCEEDED
                else '\033[31mFAIL\033[0m')
        print(f'  [{mark}] {r.scenario.name} '
              f'({r.elapsed_sec:.1f}s) {r.message}')
    print()

    return results


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Run navigation test scenarios',
    )
    parser.add_argument(
        'scenarios', nargs='*',
        help='Paths to scenario YAML files. '
             'If omitted, discovers all available scenarios.',
    )
    parser.add_argument(
        '--no-builtin', action='store_true',
        help='Exclude built-in scenarios from discovery.',
    )
    args = parser.parse_args()

    # Resolve scenario files
    if args.scenarios:
        scenario_files = [Path(s) for s in args.scenarios]
    else:
        scenario_files = discover_scenarios(
            include_builtin=not args.no_builtin,
        )

    if not scenario_files:
        print('No scenario files found.')
        sys.exit(1)

    scenarios = [load_scenario(f) for f in scenario_files]
    print(f'Loaded {len(scenarios)} scenario(s):')
    for sc in scenarios:
        print(f'  - {sc.name} ({sc.source_file})')

    rclpy.init()
    navigator = BasicNavigator()

    print('\nWaiting for Nav2 to become active...')
    navigator.waitUntilNav2Active(
        navigator='bt_navigator', localizer='robot_localization',
    )

    results = run_scenarios(navigator, scenarios)

    navigator.lifecycleShutdown()
    rclpy.shutdown()

    all_passed = all(r.status == TestStatus.SUCCEEDED for r in results)
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
