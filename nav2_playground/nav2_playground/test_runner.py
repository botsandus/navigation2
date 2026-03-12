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
Automated test runner for navigation scenarios using BasicNavigator.

Defines NavTestCase for specifying start/goal poses with timeouts, and
provides run_test / run_all_tests functions. Uses BasicNavigator from
nav2_simple_commander (no duplication of NavTester from nav2_system_tests).
"""

from dataclasses import dataclass
from enum import auto, Enum
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


class TestStatus(Enum):
    SUCCEEDED = auto()
    FAILED = auto()
    TIMED_OUT = auto()
    REJECTED = auto()


@dataclass
class NavTestCase:
    """A navigation test scenario."""

    name: str
    start_x: float
    start_y: float
    start_yaw: float
    goal_x: float
    goal_y: float
    goal_yaw: float
    timeout_sec: float = 60.0
    behavior_tree: str = ''


@dataclass
class NavTestResult:
    """Result of a single navigation test."""

    test_case: NavTestCase
    status: TestStatus
    elapsed_sec: float = 0.0
    message: str = ''


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert yaw angle to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _make_pose(x: float, y: float, yaw: float, navigator: BasicNavigator) -> PoseStamped:
    """Create a PoseStamped in the map frame."""
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


def run_test(navigator: BasicNavigator, tc: NavTestCase) -> NavTestResult:
    """Run a single navigation test case. Returns the result."""
    # Set initial pose
    start_pose = _make_pose(tc.start_x, tc.start_y, tc.start_yaw, navigator)
    navigator.setInitialPose(start_pose)

    # Wait for TF to propagate and nav2 to settle
    time.sleep(2.0)

    # Send navigation goal
    goal_pose = _make_pose(tc.goal_x, tc.goal_y, tc.goal_yaw, navigator)
    task = navigator.goToPose(goal_pose, behavior_tree=tc.behavior_tree)

    if task is None:
        return NavTestResult(
            test_case=tc,
            status=TestStatus.REJECTED,
            message='Goal was rejected by the navigator',
        )

    # Poll for completion with timeout
    t_start = time.monotonic()
    while not navigator.isTaskComplete(task=task):
        elapsed = time.monotonic() - t_start
        if elapsed > tc.timeout_sec:
            navigator.cancelTask()
            return NavTestResult(
                test_case=tc,
                status=TestStatus.TIMED_OUT,
                elapsed_sec=elapsed,
                message=f'Timed out after {elapsed:.1f}s (limit: {tc.timeout_sec}s)',
            )

    elapsed = time.monotonic() - t_start
    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        return NavTestResult(
            test_case=tc,
            status=TestStatus.SUCCEEDED,
            elapsed_sec=elapsed,
        )
    elif result == TaskResult.CANCELED:
        return NavTestResult(
            test_case=tc,
            status=TestStatus.FAILED,
            elapsed_sec=elapsed,
            message='Navigation was canceled',
        )
    else:
        return NavTestResult(
            test_case=tc,
            status=TestStatus.FAILED,
            elapsed_sec=elapsed,
            message=f'Navigation failed with result: {result}',
        )


def run_all_tests(
    navigator: BasicNavigator,
    test_cases: list[NavTestCase],
) -> list[NavTestResult]:
    """Run all test cases sequentially. Returns list of results."""
    results: list[NavTestResult] = []
    for i, tc in enumerate(test_cases, 1):
        print(f'\n[{i}/{len(test_cases)}] Running: {tc.name}')
        print(f'  Start: ({tc.start_x}, {tc.start_y}, yaw={tc.start_yaw:.2f})')
        print(f'  Goal:  ({tc.goal_x}, {tc.goal_y}, yaw={tc.goal_yaw:.2f})')
        print(f'  Timeout: {tc.timeout_sec}s')

        result = run_test(navigator, tc)
        results.append(result)

        status_str = result.status.name
        if result.status == TestStatus.SUCCEEDED:
            print(f'  Result: \033[32m{status_str}\033[0m ({result.elapsed_sec:.1f}s)')
        else:
            print(f'  Result: \033[31m{status_str}\033[0m ({result.elapsed_sec:.1f}s)')
            if result.message:
                print(f'  Message: {result.message}')

    # Summary
    passed = sum(1 for r in results if r.status == TestStatus.SUCCEEDED)
    total = len(results)
    print(f'\n{"=" * 60}')
    print(f'Results: {passed}/{total} passed')
    print(f'{"=" * 60}')
    for r in results:
        mark = '\033[32mPASS\033[0m' if r.status == TestStatus.SUCCEEDED else '\033[31mFAIL\033[0m'
        print(f'  [{mark}] {r.test_case.name} ({r.elapsed_sec:.1f}s) {r.message}')
    print()

    return results


# Default example test cases (used when running as a standalone script)
DEFAULT_TEST_CASES = [
    NavTestCase(
        name='straight_line_forward',
        start_x=-2.0, start_y=-0.5, start_yaw=0.0,
        goal_x=0.0, goal_y=-0.5, goal_yaw=0.0,
        timeout_sec=30.0,
    ),
    NavTestCase(
        name='diagonal_move',
        start_x=-2.0, start_y=-0.5, start_yaw=0.0,
        goal_x=0.0, goal_y=1.0, goal_yaw=math.pi / 2,
        timeout_sec=45.0,
    ),
]


def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()

    print('Waiting for Nav2 to become active...')
    # Pass localizer='robot_localization' to skip waiting for AMCL/localizer
    # (loopback simulator provides map->odom->base_footprint TF directly)
    navigator.waitUntilNav2Active(navigator='bt_navigator',
                                  localizer='robot_localization')

    results = run_all_tests(navigator, DEFAULT_TEST_CASES)

    navigator.lifecycleShutdown()
    rclpy.shutdown()

    all_passed = all(r.status == TestStatus.SUCCEEDED for r in results)
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
