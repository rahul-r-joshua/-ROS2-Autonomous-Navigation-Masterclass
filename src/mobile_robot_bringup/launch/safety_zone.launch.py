#!/usr/bin/env python3
"""
================================================================================
Mobile Robot Safety Zone Launcher
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Launches the Safety Zone Controller (Active LiDAR collision avoidance).
Supports switching between C++ binary (default) and Python script via use_cpp:=true/false.
================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Launch C++ safety zone node if true, else Python node'
    )

    # C++ Safety Zone Node
    safety_cpp_node = Node(
        package='mobile_robot_bringup',
        executable='safety_zone_controller_cpp',
        name='safety_zone_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_cpp'))
    )

    # Python Safety Zone Node
    safety_py_node = Node(
        package='mobile_robot_bringup',
        executable='safety_zone_controller.py',
        name='safety_zone_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=UnlessCondition(LaunchConfiguration('use_cpp'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_cpp_arg,
        safety_cpp_node,
        safety_py_node
    ])
