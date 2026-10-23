#!/usr/bin/env python3
"""
================================================================================
planner.launch.py
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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_planning')
    default_params = os.path.join(pkg_share, 'config', 'planner_server.yaml')

    planner = LaunchConfiguration('planner')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_planner_cmd = DeclareLaunchArgument(
        'planner',
        default_value='cpp',
        description='Planner to run: cpp, a_star.py, or dijkstra.py'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the ROS2 parameters file for planner_server'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the planner lifecycle stack'
    )

    # C++ Nav2 Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", planner, "' == 'cpp'"]))
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['planner_server']}
        ],
        condition=IfCondition(PythonExpression(["'", planner, "' == 'cpp'"]))
    )

    # Python A* Planner Node
    a_star_node = Node(
        package='mobile_robot_planning',
        executable='a_star_planner.py',
        name='a_star_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/costmap/costmap', '/map')],
        condition=IfCondition(PythonExpression(["'", planner, "' == 'a_star.py' or '", planner, "' == 'a_star'"]))
    )

    # Python Dijkstra Planner Node
    dijkstra_node = Node(
        package='mobile_robot_planning',
        executable='dijkstra_planner.py',
        name='dijkstra_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/costmap/costmap', '/map')],
        condition=IfCondition(PythonExpression(["'", planner, "' == 'dijkstra.py' or '", planner, "' == 'dijkstra'"]))
    )

    return LaunchDescription([
        declare_planner_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        planner_server_node,
        lifecycle_manager_node,
        a_star_node,
        dijkstra_node
    ])
