#!/usr/bin/env python3
"""
================================================================================
Nav2 Navigation Launch File (Mobile Robot Decoupled Architecture)
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Launches Planners, Controllers, Behavior Recovery, BT Navigator,
and dedicated lifecycle_manager_navigation.
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_nav = get_package_share_directory('mobile_robot_navigation')
    default_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    default_bt_xml = os.path.join(
        pkg_nav, 'behavior_tree', 'simple_navigation_w_replanning_and_recoveries.xml'
    )
    if not os.path.exists(default_bt_xml):
        default_bt_xml = "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    # Core Navigation lifecycle nodes (Decoupled from map_server & amcl)
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    remappings = [
        ('/cmd_vel', '/cmd_vel_nav')
    ]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'default_nav_to_pose_bt_xml': default_bt_xml_filename,
        'default_bt_xml_filename': default_bt_xml_filename
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=default_params_file, description='Full path to Nav2 parameter file')
    declare_bt_xml_cmd = DeclareLaunchArgument('default_bt_xml_filename', default_value=default_bt_xml, description='Full path to Behavior Tree XML file')
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack')

    nav2_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    nav2_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    nav2_behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    nav2_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes},
            {'bond_timeout': 10.0},
            {'attempt_respawn_reconnection': True}
        ]
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_bt_xml_cmd,
        declare_autostart_cmd,
        nav2_controller_server,
        nav2_planner_server,
        nav2_behaviors,
        nav2_bt_navigator,
        nav2_waypoint_follower,
        nav2_lifecycle_manager
    ])

