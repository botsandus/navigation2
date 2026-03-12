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
Gazebo backend.

Generates launch actions that bring up the full navigation stack with the
Gazebo simulator (physics, sensors, robot model).

Mirrors the pattern used by ``nav2_system_tests/src/system/test_system_launch.py``
but is reusable from the shared test framework.

Launches:
  - Gazebo (``gz sim``) with the TB3 sandbox world
  - TB3 robot spawn
  - robot_state_publisher
  - Full nav2 bringup (bringup_launch.py)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (AppendEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_nav_actions(
    *,
    params_file: str,
    map_yaml: str = '',
    bt_xml: str = '',
    controller_plugin: str = '',
    planner_plugin: str = '',
    use_astar: bool = False,
    groot_monitoring: bool = False,
    inflation_layer_plugin: str = '',
    extra_actions: list | None = None,
    autostart: bool = True,
) -> list:
    """Return a list of launch actions for the Gazebo nav stack."""
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    bringup_dir = get_package_share_directory('nav2_bringup')

    world_sdf_xacro = os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')
    robot_sdf = os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')

    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf) as f:
        robot_description = f.read()

    if not map_yaml:
        map_yaml = os.path.join(bringup_dir, 'maps', 'tb3_sandbox.yaml')

    bt_navigator_xml = ''
    if bt_xml:
        bt_navigator_xml = os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', bt_xml,
        )

    # Param substitutions (same approach as nav2_system_tests)
    param_rewrites: dict[str, str] = {}
    if use_astar:
        param_rewrites['use_astar'] = 'True'
    if groot_monitoring:
        param_rewrites['enable_groot_monitoring'] = 'True'
    if inflation_layer_plugin:
        param_rewrites[
            'local_costmap.local_costmap.ros__parameters.inflation_layer.plugin'
        ] = inflation_layer_plugin
        param_rewrites[
            'global_costmap.global_costmap.ros__parameters.inflation_layer.plugin'
        ] = inflation_layer_plugin
    if planner_plugin:
        param_rewrites[
            'planner_server.ros__parameters.GridBased.plugin'
        ] = planner_plugin
    if controller_plugin:
        param_rewrites[
            'controller_server.ros__parameters.FollowPath.plugin'
        ] = controller_plugin

    context = LaunchContext()
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_rewrites,
        convert_types=True,
    )
    new_yaml = configured_params.perform(context)

    actions = [
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models'),
        ),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(sim_dir)).parent.resolve()),
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', world_sdf_xacro],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py'),
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'robot_sdf': robot_sdf,
                'x_pose': '-2.0',
                'y_pose': '-0.5',
                'z_pose': '0.01',
                'roll': '0.0',
                'pitch': '0.0',
                'yaw': '0.0',
            }.items(),
        ),
    ]

    if extra_actions:
        actions.extend(extra_actions)

    actions.extend([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'bringup_launch.py'),
            ),
            launch_arguments={
                'namespace': '',
                'map': map_yaml,
                'use_sim_time': 'True',
                'params_file': new_yaml,
                'bt_xml_file': bt_navigator_xml,
                'use_composition': 'False',
                'autostart': str(autostart),
            }.items(),
        ),
    ])

    return actions
