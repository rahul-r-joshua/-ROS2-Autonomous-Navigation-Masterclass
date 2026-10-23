#!/usr/bin/env python3
"""
================================================================================
Joystick Teleop Launch File (matching rocker_controller architecture)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_pkg = get_package_share_directory('mobile_robot_controller')
    twist_mux_pkg = get_package_share_directory('twist_mux')

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulated time"
    )

    cmd_vel_out_arg = DeclareLaunchArgument(
        name="cmd_vel_out",
        default_value="/cmd_vel",
        description="Output topic for twist_mux (default: /cmd_vel; /cmd_vel_raw when safety controller is active)"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(controller_pkg, "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    # teleop node (reads /joy, checks deadman button 4 LB, outputs /joy_vel)
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            os.path.join(controller_pkg, "config", "joy_teleop.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        remappings=[('/cmd_vel', '/joy_vel')]
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings=[("/cmd_vel_out", LaunchConfiguration("cmd_vel_out"))],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            os.path.join(controller_pkg, "config", "twist_mux_locks.yaml"),
            os.path.join(controller_pkg, "config", "twist_mux_topics.yaml"),
        ]
    )

    twist_marker_node = Node(
        package="twist_mux",
        executable="twist_marker",
        output="screen",
        remappings=[("/twist", LaunchConfiguration("cmd_vel_out"))],
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "frame_id": "base_link",
            "scale": 1.0,
            "vertical_position": 2.0
        }]
    )

    twist_relay_node = Node(
        package="mobile_robot_controller",
        executable="twist_relay.py",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        cmd_vel_out_arg,
        joy_node,
        teleop_node,
        twist_mux_node,
        twist_marker_node,
        twist_relay_node,
    ])
