#!/usr/bin/env python3
"""
================================================================================
controller.launch.py
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
    pkg_share = get_package_share_directory('mobile_robot_motion')
    default_params = os.path.join(pkg_share, 'config', 'controller_server.yaml')

    motion = LaunchConfiguration('motion')
    controller = LaunchConfiguration('controller')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_motion_cmd = DeclareLaunchArgument(
        'motion',
        default_value='cpp',
        description='Execution track: cpp (Nav2 action plugin) or py (standalone node)'
    )

    declare_controller_cmd = DeclareLaunchArgument(
        'controller',
        default_value='pure_pursuit',
        description='Algorithm for Python standalone mode: pure_pursuit or pd_motion'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to parameter file for controller_server'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the controller lifecycle stack'
    )

    # C++ Nav2 Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", motion, "' == 'cpp'"]))
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['controller_server']}
        ],
        condition=IfCondition(PythonExpression(["'", motion, "' == 'cpp'"]))
    )

    # Python Pure Pursuit Node
    pure_pursuit_node = Node(
        package='mobile_robot_motion',
        executable='pure_pursuit.py',
        name='pure_pursuit_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'max_linear_velocity': 0.40},
            {'max_angular_velocity': 1.80},
            {'look_ahead_distance': 0.40}
        ],
        condition=IfCondition(PythonExpression(["'", motion, "' == 'py' and ('", controller, "' == 'pure_pursuit' or '", controller, "' == 'pure_pursuit.py')"]))
    )

    # Python PD Motion Node
    pd_motion_node = Node(
        package='mobile_robot_motion',
        executable='pd_motion.py',
        name='pd_motion_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'max_linear_velocity': 0.40},
            {'max_angular_velocity': 1.80},
            {'kp': 2.5},
            {'step_size': 0.25}
        ],
        condition=IfCondition(PythonExpression(["'", motion, "' == 'py' and ('", controller, "' == 'pd_motion' or '", controller, "' == 'pd_motion.py')"]))
    )

    return LaunchDescription([
        declare_motion_cmd,
        declare_controller_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        controller_server_node,
        lifecycle_manager_node,
        pure_pursuit_node,
        pd_motion_node
    ])
