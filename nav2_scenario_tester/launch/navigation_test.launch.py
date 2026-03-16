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

"""
Top-level launch file for nav2 navigation integration tests.

Includes sim backend (loopback or gazebo), map_server, and minimal nav2 nodes.
Intended to be included from user test launch files via IncludeLaunchDescription.

Usage from another package:
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_scenario_tester'),
                'launch', 'navigation_test.launch.py'
            )
        ),
        launch_arguments={
            'sim_type': 'loopback',
            'params_file': '/path/to/my_params.yaml',
        }.items(),
    )
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import LaunchConfigAsBool


def generate_launch_description() -> LaunchDescription:
    # Directories
    pkg_dir = get_package_share_directory('nav2_scenario_tester')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    # Launch configurations
    sim_type = LaunchConfiguration('sim_type')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfigAsBool('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    log_level = LaunchConfiguration('log_level')

    # Gazebo-specific
    world = LaunchConfiguration('world')
    robot_sdf = LaunchConfiguration('robot_sdf')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    # Conditions
    is_loopback = PythonExpression(["'", sim_type, "' == 'loopback'"])
    is_gazebo = PythonExpression(["'", sim_type, "' == 'gazebo'"])

    # Default paths
    default_params = os.path.join(pkg_dir, 'config', 'default_test_params.yaml')
    default_map = os.path.join(pkg_dir, 'maps', 'empty.yaml')
    default_world = os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')
    default_robot_sdf = os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')
    default_urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')

    # ── Declare arguments ──────────────────────────────────────────────
    declare_sim_type = DeclareLaunchArgument(
        'sim_type', default_value='loopback',
        description="Simulation backend: 'loopback' or 'gazebo'",
    )
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
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Full path to Gazebo world SDF/xacro',
    )
    declare_robot_sdf = DeclareLaunchArgument(
        'robot_sdf', default_value=default_robot_sdf,
        description='Full path to robot SDF for Gazebo spawn',
    )
    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='-2.0', description='Robot spawn X',
    )
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='-0.5', description='Robot spawn Y',
    )
    declare_z_pose = DeclareLaunchArgument(
        'z_pose', default_value='0.01', description='Robot spawn Z',
    )
    declare_yaw = DeclareLaunchArgument(
        'yaw', default_value='0.0', description='Robot spawn yaw',
    )

    # ── Loopback simulation group ──────────────────────────────────────
    loopback_group = GroupAction(
        condition=IfCondition(is_loopback),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            # Loopback simulator — publishes /clock, so must NOT use sim time
            Node(
                package='nav2_loopback_sim',
                executable='loopback_simulator',
                name='loopback_simulator',
                output='screen',
                parameters=[params_file, {
                    'use_sim_time': False,
                    'scan_frame_id': 'base_scan',
                    'base_frame_id': 'base_link',
                    'publish_scan': False,
                }],
            ),
            # Map server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[params_file, {'yaml_filename': map_yaml}],
            ),
            # Lifecycle manager for map_server (loopback doesn't need AMCL)
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_server'],
                }],
            ),
        ],
    )

    # ── Gazebo simulation group ────────────────────────────────────────
    gazebo_group = GroupAction(
        condition=IfCondition(is_gazebo),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models'),
            ),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                str(Path(os.path.join(sim_dir)).parent.resolve()),
            ),
            # Gazebo simulator (headless)
            ExecuteProcess(
                cmd=['gz', 'sim', '-r', '-s', world],
                output='screen',
            ),
            # Spawn robot
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py'),
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'robot_sdf': robot_sdf,
                    'x_pose': x_pose,
                    'y_pose': y_pose,
                    'z_pose': z_pose,
                    'roll': '0.0',
                    'pitch': '0.0',
                    'yaw': yaw,
                }.items(),
            ),
            # Static map→odom identity (no AMCL needed)
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            ),
            # Robot state publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': _read_urdf(default_urdf),
                }],
            ),
            # Map server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[params_file, {'yaml_filename': map_yaml}],
            ),
            # Lifecycle manager for map_server
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_server'],
                }],
            ),
        ],
    )

    # ── Nav2 nodes (shared by both sim types) ──────────────────────────
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

    # ── Assemble ───────────────────────────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))

    # Declare all arguments
    for decl in [
        declare_sim_type, declare_params_file, declare_map,
        declare_use_sim_time, declare_namespace, declare_log_level,
        declare_world, declare_robot_sdf,
        declare_x_pose, declare_y_pose, declare_z_pose, declare_yaw,
    ]:
        ld.add_action(decl)

    # Sim backends (mutually exclusive via conditions)
    ld.add_action(loopback_group)
    ld.add_action(gazebo_group)

    # Nav2 nodes
    ld.add_action(nav_nodes)

    return ld


def _read_urdf(urdf_path: str) -> str:
    """Read URDF file contents for robot_state_publisher."""
    with open(urdf_path, 'r') as f:
        return f.read()
