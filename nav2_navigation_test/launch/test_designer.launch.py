# Copyright (c) 2024 Open Navigation LLC
# Licensed under the Apache License, Version 2.0

"""Launch file for the Nav Test Designer — map + RViz with the design panel."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav2_navigation_test')

    map_yaml = LaunchConfiguration('map')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/home/ubuntu/maps/active_map/map.yaml',
        description='Full path to map YAML file',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'test_designer.rviz'),
        description='Full path to RViz config file',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    ld = LaunchDescription()
    ld.add_action(declare_map)
    ld.add_action(declare_rviz_config)
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(rviz)
    return ld
