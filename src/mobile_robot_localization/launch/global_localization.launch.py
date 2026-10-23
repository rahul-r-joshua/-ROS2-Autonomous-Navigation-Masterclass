#!/usr/bin/env python3
"""
================================================================================
Localization Launch File (AMCL + Map Server Lifecycle)
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mapping_pkg = get_package_share_directory('mobile_robot_mapping')
    loc_pkg = get_package_share_directory('mobile_robot_localization')

    default_map_path = os.path.join(mapping_pkg, 'maps', 'nav_challenge_map.yaml')
    default_amcl_config = os.path.join(loc_pkg, 'config', 'amcl.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    amcl_config = LaunchConfiguration('amcl_config')
    autostart = LaunchConfiguration('autostart')

    lifecycle_nodes = ['map_server', 'amcl']

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map YAML file to load'
    )

    declare_amcl_config_cmd = DeclareLaunchArgument(
        'amcl_config',
        default_value=default_amcl_config,
        description='Full path to AMCL parameter YAML file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the localization stack'
    )

    nav2_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}]
    )

    nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config,
                    {'use_sim_time': use_sim_time}]
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_cmd,
        declare_amcl_config_cmd,
        declare_autostart_cmd,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager
    ])
