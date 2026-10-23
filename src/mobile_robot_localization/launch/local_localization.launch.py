#!/usr/bin/env python3
"""
================================================================================
local_localization.launch.py
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
================================================================================
"""
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
import os

def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    use_python = LaunchConfiguration("use_python")
    use_sim_time = LaunchConfiguration("use_sim_time")

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0","--z", "0.103",
                   "--qx", "1", "--qy", "0", "--qz", "0", "--qw", "0",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("mobile_robot_localization"), "config", "ekf.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    imu_republisher_py = Node(
        package="mobile_robot_localization",
        executable="imu_republisher.py",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_python),
    )

    imu_republisher_cpp = Node(
        package="mobile_robot_localization",
        executable="imu_republisher",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_python),
    )

    return LaunchDescription([
        use_python_arg,
        use_sim_time_arg,
        static_transform_publisher,
        robot_localization,
        imu_republisher_py,
        imu_republisher_cpp,   
    ])
