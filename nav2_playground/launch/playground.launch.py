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
Playground launch file for testing navigation without a physics simulator.

Launches:
  - Map server (with lifecycle manager)
  - Loopback simulator (cmd_vel -> odom/TF, fake laser scans)
  - Navigation nodes: controller_server, planner_server, smoother_server,
    behavior_server, bt_navigator (with lifecycle manager)
  - RViz (optional)

No velocity smoother, collision monitor, AMCL, or composition.
Set pose via RViz "2D Pose Estimate" or publish to /initialpose.
Send goals via RViz "2D Nav Goal" or publish to /goal_pose.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import LaunchConfigAsBool, RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory('nav2_bringup')
    loopback_sim_dir = get_package_share_directory('nav2_loopback_sim')

    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfigAsBool('autostart')
    use_sim_time = LaunchConfigAsBool('use_sim_time')
    use_rviz = LaunchConfigAsBool('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # --- Declare launch arguments ---

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        description='Full path to the ROS2 parameters file (required)',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock from loopback simulator',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RViz'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RViz config file',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    # --- Map server ---

    start_map_server = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map_server',
                output='screen',
                parameters=[
                    configured_params,
                    {'autostart': autostart},
                    {'node_names': ['map_server']},
                ],
            ),
        ]
    )

    # --- Loopback simulator (no fake laser scan) ---

    loopback_sim_cmd = GroupAction(
        actions=[
            SetParameter(name='publish_scan', value=False),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(loopback_sim_dir, 'loopback_simulation.launch.py')
                ),
                launch_arguments={
                    'params_file': params_file,
                }.items(),
            ),
        ]
    )

    # --- Navigation nodes (standalone, no composition) ---

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    start_navigation = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    configured_params,
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                ],
            ),
        ]
    )

    # --- RViz ---

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time),
            ('rviz_config', rviz_config_file),
        ],
    )

    # --- Assemble launch description ---

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(start_map_server)
    ld.add_action(loopback_sim_cmd)
    ld.add_action(start_navigation)
    ld.add_action(rviz_cmd)

    return ld
