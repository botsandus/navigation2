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

#
# Register a Nav2 BT + planner + controller integration test.
#
# This macro generates a launch_testing test that spins up a minimal Nav2
# stack (via navigation_test.launch.py) with the specified simulation
# backend and then runs the NavTestRunner CLI to navigate from START_POSE
# to GOAL_POSE using the given PARAMS_FILE.
#
# Usage:
#   find_package(nav2_scenario_tester REQUIRED)  # auto-loaded via CONFIG_EXTRAS
#
#   nav2_scenario_add_test(my_mppi_loopback_test
#     SIM_TYPE loopback
#     PARAMS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/config/my_mppi_params.yaml
#     MAP ${CMAKE_CURRENT_SOURCE_DIR}/maps/my_map.yaml
#     START_POSE "-2.0;-0.5;0.0"
#     GOAL_POSE "2.0;0.5;0.0"
#     TIMEOUT 120
#   )
#
# Arguments:
#   NAME         (positional) - CTest name for this test
#   SIM_TYPE     - Simulation backend: "loopback" or "gazebo" (default: loopback)
#   PARAMS_FILE  - Absolute path to a Nav2 params YAML file (optional, uses defaults)
#   MAP          - Absolute path to a map YAML file (optional, uses nav2_bringup default)
#   START_POSE   - Semicolon-separated "x;y;yaw" for the start pose (default: "-2.0;-0.5;0.0")
#   GOAL_POSE    - Semicolon-separated "x;y;yaw" for the goal pose (default: "0.0;2.0;0.0")
#   TIMEOUT      - Test timeout in seconds (default: 120)
#   BT_XML_FILE  - Absolute path to a behavior tree XML file (optional)
#   TOLERANCE    - Goal distance tolerance in meters (default: 0.5)
#
# @public
#
function(nav2_scenario_add_test NAV_TEST_NAME)
  cmake_parse_arguments(NAV_TEST
    ""
    "SIM_TYPE;PARAMS_FILE;MAP;START_POSE;GOAL_POSE;TIMEOUT;BT_XML_FILE;TOLERANCE"
    ""
    ${ARGN}
  )

  # ── Defaults ──────────────────────────────────────────────────────

  if(NOT NAV_TEST_SIM_TYPE)
    set(NAV_TEST_SIM_TYPE "loopback")
  endif()

  if(NOT NAV_TEST_START_POSE)
    set(NAV_TEST_START_POSE "-2.0;-0.5;0.0")
  endif()

  if(NOT NAV_TEST_GOAL_POSE)
    set(NAV_TEST_GOAL_POSE "0.0;2.0;0.0")
  endif()

  if(NOT NAV_TEST_TIMEOUT)
    set(NAV_TEST_TIMEOUT 120)
  endif()

  if(NOT NAV_TEST_TOLERANCE)
    set(NAV_TEST_TOLERANCE "0.5")
  endif()

  # ── Parse pose components ─────────────────────────────────────────

  string(REPLACE ";" "," _start_csv "${NAV_TEST_START_POSE}")
  string(REPLACE ";" "," _goal_csv "${NAV_TEST_GOAL_POSE}")

  list(GET NAV_TEST_START_POSE 0 _start_x)
  list(GET NAV_TEST_START_POSE 1 _start_y)
  list(LENGTH NAV_TEST_START_POSE _start_len)
  if(_start_len GREATER 2)
    list(GET NAV_TEST_START_POSE 2 _start_yaw)
  else()
    set(_start_yaw "0.0")
  endif()

  list(GET NAV_TEST_GOAL_POSE 0 _goal_x)
  list(GET NAV_TEST_GOAL_POSE 1 _goal_y)
  list(LENGTH NAV_TEST_GOAL_POSE _goal_len)
  if(_goal_len GREATER 2)
    list(GET NAV_TEST_GOAL_POSE 2 _goal_yaw)
  else()
    set(_goal_yaw "0.0")
  endif()

  # ── Build the test launch file content ────────────────────────────
  # We generate a Python launch_testing file that:
  # 1. Includes the navigation_test.launch.py from nav2_scenario_tester
  # 2. Runs the NavTestRunner CLI as a test process
  # 3. Checks exit code

  set(_test_file "${CMAKE_CURRENT_BINARY_DIR}/nav_test_${NAV_TEST_NAME}.py")

  # Build launch arguments for the navigation stack
  set(_launch_args "")
  string(APPEND _launch_args "            ('sim_type', '${NAV_TEST_SIM_TYPE}'),\n")
  string(APPEND _launch_args "            ('use_sim_time', 'True'),\n")
  string(APPEND _launch_args "            ('autostart', 'True'),\n")
  string(APPEND _launch_args "            ('x_pose', '${_start_x}'),\n")
  string(APPEND _launch_args "            ('y_pose', '${_start_y}'),\n")
  string(APPEND _launch_args "            ('yaw', '${_start_yaw}'),\n")

  if(NAV_TEST_PARAMS_FILE)
    string(APPEND _launch_args "            ('params_file', '${NAV_TEST_PARAMS_FILE}'),\n")
  endif()

  if(NAV_TEST_MAP)
    string(APPEND _launch_args "            ('map', '${NAV_TEST_MAP}'),\n")
  endif()

  if(NAV_TEST_BT_XML_FILE)
    string(APPEND _launch_args "            ('bt_xml_file', '${NAV_TEST_BT_XML_FILE}'),\n")
  endif()

  # Build CLI args for the test_runner
  set(_runner_args
    "--start-x" "${_start_x}"
    "--start-y" "${_start_y}"
    "--start-yaw" "${_start_yaw}"
    "--goal-x" "${_goal_x}"
    "--goal-y" "${_goal_y}"
    "--goal-yaw" "${_goal_yaw}"
    "--timeout" "${NAV_TEST_TIMEOUT}"
    "--tolerance" "${NAV_TEST_TOLERANCE}"
  )

  if(NAV_TEST_SIM_TYPE STREQUAL "gazebo")
    list(APPEND _runner_args "--wait-for-initial-pose")
  endif()

  # Convert runner args list to a Python list literal
  set(_runner_args_py "")
  foreach(_arg IN LISTS _runner_args)
    if(_runner_args_py)
      string(APPEND _runner_args_py ", ")
    endif()
    string(APPEND _runner_args_py "'${_arg}'")
  endforeach()

  set(_py_src "PythonLaunchDescriptionSource")
  file(WRITE "${_test_file}"
"# Auto-generated by nav2_scenario_add_test(${NAV_TEST_NAME})
import os
import sys
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import ${_py_src}
import launch_testing
import launch_testing.actions

from ament_index_python.packages import get_package_share_directory


def generate_test_description():
    nav_test_dir = get_package_share_directory('nav2_scenario_tester')
    launch_file = os.path.join(nav_test_dir, 'launch', 'navigation_test.launch.py')

    nav_stack = IncludeLaunchDescription(
        ${_py_src}(launch_file),
        launch_arguments=[
${_launch_args}        ],
    )

    test_runner = launch.actions.ExecuteProcess(
        cmd=[
            sys.executable, '-m', 'nav2_scenario_tester.test_runner',
            ${_runner_args_py},
        ],
        output='screen',
    )

    return launch.LaunchDescription([
        nav_stack,
        launch.actions.TimerAction(period=5.0, actions=[test_runner]),
        launch_testing.actions.ReadyToTest(),
    ]), {'test_runner': test_runner}


class TestNavigationResult(unittest.TestCase):

    def test_navigation_completed(self, proc_info, test_runner):
        proc_info.assertWaitForShutdown(process=test_runner, timeout=${NAV_TEST_TIMEOUT})


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info, test_runner):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            process=test_runner,
        )
"
  )

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    "${_test_file}"
    TARGET "${NAV_TEST_NAME}"
    TIMEOUT ${NAV_TEST_TIMEOUT}
  )
endfunction()
