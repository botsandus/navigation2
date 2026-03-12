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
Playground test runner.

Thin wrapper around nav2_nav_tester that defines a few default scenarios
and exposes a ``playground_test_runner`` console-script entry point.
"""

import math
import sys

from nav2_nav_tester.runner import run_scenarios, TestStatus
from nav2_nav_tester.scenario import Scenario
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

# Default example scenarios (used when running as a standalone script)
DEFAULT_TEST_CASES = [
    Scenario(
        name='straight_line_forward',
        description='Straight line forward',
        start_x=-2.0, start_y=-0.5, start_yaw=0.0,
        goal_x=0.0, goal_y=-0.5, goal_yaw=0.0,
        timeout_sec=30.0,
    ),
    Scenario(
        name='diagonal_move',
        description='Diagonal with heading change',
        start_x=-2.0, start_y=-0.5, start_yaw=0.0,
        goal_x=0.0, goal_y=1.0, goal_yaw=math.pi / 2,
        timeout_sec=45.0,
    ),
]


def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()

    print('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active(navigator='bt_navigator',
                                  localizer='robot_localization')

    results = run_scenarios(navigator, DEFAULT_TEST_CASES)

    navigator.lifecycleShutdown()
    rclpy.shutdown()

    all_passed = all(r.status == TestStatus.SUCCEEDED for r in results)
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
