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
Loopback backend.

Generates a LaunchDescription that brings up the full navigation stack
using the loopback simulator (no physics engine).

Launches:
  - map_server + lifecycle manager
  - nav2_loopback_sim (with publish_scan=False)
  - controller_server, planner_server, behavior_server, bt_navigator
  - lifecycle_manager_navigation
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_nav_actions(
    *,
    params_file: str,
    map_yaml: str = '',
    autostart: bool = True,
) -> list:
    """Return a list of launch actions for the loopback nav stack."""
    bringup_dir = get_package_share_directory('nav2_bringup')
    loopback_sim_dir = get_package_share_directory('nav2_loopback_sim')

    if not map_yaml:
        map_yaml = os.path.join(bringup_dir, 'maps', 'tb3_sandbox.yaml')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # -- Map server --
    map_server_group = GroupAction(actions=[
        SetParameter('use_sim_time', True),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': map_yaml}],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[
                configured_params,
                {'autostart': autostart, 'node_names': ['map_server']},
            ],
        ),
    ])

    # -- Loopback simulator (no fake laser scan) --
    loopback_sim = GroupAction(actions=[
        SetParameter(name='publish_scan', value=False),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loopback_sim_dir, 'loopback_simulation.launch.py')
            ),
            launch_arguments={'params_file': params_file}.items(),
        ),
    ])

    # -- Navigation nodes --
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    nav_group = GroupAction(actions=[
        SetParameter('use_sim_time', True),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                configured_params,
                {'autostart': autostart, 'node_names': lifecycle_nodes},
            ],
        ),
    ])

    return [map_server_group, loopback_sim, nav_group]
