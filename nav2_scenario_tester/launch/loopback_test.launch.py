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
Launch file for nav2 navigation integration tests with loopback simulation.

Spins up a loopback simulator, map_server, and minimal nav2 nodes.
Intended to be included from user test launch files via IncludeLaunchDescription.

Usage from another package:
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_scenario_tester'),
                'launch', 'loopback_test.launch.py'
            )
        ),
        launch_arguments={
            'params_file': '/path/to/my_params.yaml',
        }.items(),
    )
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import LaunchConfigAsBool


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory('nav2_scenario_tester')

    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfigAsBool('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    log_level = LaunchConfiguration('log_level')

    default_params = os.path.join(pkg_dir, 'config', 'default_test_params.yaml')
    default_map = os.path.join(pkg_dir, 'maps', 'empty.yaml')

    # ── Declare arguments ──────────────────────────────────────────────
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the ROS2 parameters file',
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map YAML file',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock',
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace',
    )
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz for visual debugging',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'test_debug.rviz'),
        description='Full path to RViz config file',
    )

    # ── Loopback simulation ────────────────────────────────────────────
    set_sim_time = SetParameter('use_sim_time', use_sim_time)

    # Loopback simulator — publishes /clock, so must NOT use sim time
    loopback_simulator = Node(
        package='nav2_loopback_sim',
        executable='loopback_simulator',
        name='loopback_simulator',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': False,
            'scan_frame_id': 'base_scan',
            'base_frame_id': 'base_link',
            'publish_scan': False,
            'update_duration': 0.03,
        }],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml}],
    )

    vector_object_server = Node(
        package='nav2_map_server',
        executable='vector_object_server',
        name='vector_object_server',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'vector_object_server'],
        }],
    )

    # ── Nav2 nodes ─────────────────────────────────────────────────────
    nav_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'nav_nodes.launch.py'),
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params_file,
            'log_level': log_level,
        }.items(),
    )

    # ── Optional RViz ──────────────────────────────────────────────────
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    # ── Assemble ───────────────────────────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))

    for decl in [
        declare_params_file, declare_map, declare_use_sim_time,
        declare_namespace, declare_log_level,
        declare_rviz, declare_rviz_config,
    ]:
        ld.add_action(decl)

    ld.add_action(set_sim_time)
    ld.add_action(loopback_simulator)
    ld.add_action(map_server)
    ld.add_action(vector_object_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(nav_nodes)
    ld.add_action(rviz)

    return ld
