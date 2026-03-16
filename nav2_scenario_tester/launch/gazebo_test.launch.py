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
Launch file for nav2 navigation integration tests with Gazebo simulation.

Spins up Gazebo, spawns a robot, starts map_server, and minimal nav2 nodes.
Intended to be included from user test launch files via IncludeLaunchDescription.

Usage from another package:
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_scenario_tester'),
                'launch', 'gazebo_test.launch.py'
            )
        ),
        launch_arguments={
            'params_file': '/path/to/my_params.yaml',
        }.items(),
    )
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import LaunchConfigAsBool


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory('nav2_scenario_tester')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfigAsBool('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    log_level = LaunchConfiguration('log_level')

    world = LaunchConfiguration('world')
    robot_sdf = LaunchConfiguration('robot_sdf')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    default_params = os.path.join(pkg_dir, 'config', 'default_test_params.yaml')
    default_map = os.path.join(pkg_dir, 'maps', 'empty.yaml')
    default_world = os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')
    default_robot_sdf = os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')
    default_urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')

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
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run Gazebo in headless (server-only) mode',
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

    headless = LaunchConfiguration('headless')

    # ── Gazebo simulation ──────────────────────────────────────────────
    set_sim_time = SetParameter('use_sim_time', use_sim_time)

    append_gz_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models'),
    )
    append_gz_parent = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(sim_dir).parent.resolve()),
    )

    # Gazebo simulator
    gz_cmd = ['gz', 'sim', '-r']
    gz_cmd.extend([PythonExpression(["'-s' if '", headless, "' == 'true' else ''"])])
    gz_cmd.append(world)
    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
    )

    # Spawn robot
    spawn_robot = IncludeLaunchDescription(
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
    )

    # Static map→odom identity (no AMCL needed)
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': _read_urdf(default_urdf),
        }],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
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
        declare_world, declare_robot_sdf,
        declare_x_pose, declare_y_pose, declare_z_pose, declare_yaw,
        declare_headless,
        declare_rviz, declare_rviz_config,
    ]:
        ld.add_action(decl)

    ld.add_action(set_sim_time)
    ld.add_action(append_gz_models)
    ld.add_action(append_gz_parent)
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(map_to_odom)
    ld.add_action(robot_state_publisher)
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(nav_nodes)
    ld.add_action(rviz)

    return ld


def _read_urdf(urdf_path: str) -> str:
    """Read URDF file contents for robot_state_publisher."""
    with open(urdf_path, 'r') as f:
        return f.read()
