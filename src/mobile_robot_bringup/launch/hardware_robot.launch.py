#!/usr/bin/env python3
"""
================================================================================
Hardware Robot Bringup Launch File (Raspberry Pi / Physical Robot)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Launches the complete hardware stack for the real 4-wheel mobile robot:
 1. Robot State Publisher (URDF/Xacro with 4 wheels, LiDAR, Camera, IMU)
 2. Differential Drive Kinematics Controller (C++ or Python)
 3. Serial Hardware Bridge (Communicates with Arduino / ESP32 over USB Serial,
    reads encoder ticks and IMU data, sends motor PWM commands)
 4. Twist Mux (Prioritized velocity multiplexer with emergency /e_stop lock)
 5. Optional RViz2 (Default: false, for headless Raspberry Pi operation)
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('mobile_robot_description')
    ctrl_pkg = get_package_share_directory('mobile_robot_controller')
    bringup_pkg = get_package_share_directory('mobile_robot_bringup')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'mobile_robot.urdf.xacro')
    rviz_file = os.path.join(desc_pkg, 'rviz', 'simulation.rviz')
    controller_params = os.path.join(ctrl_pkg, 'config', 'controller_params.yaml')

    # Launch Arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Microcontroller serial port (e.g. /dev/ttyUSB0 or /dev/ttyACM0)'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial communication baud rate'
    )

    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Set to true for C++ controller, false for Python controller'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz2 (set false on headless Raspberry Pi, true on laptop)'
    )

    use_twist_mux_arg = DeclareLaunchArgument(
        'use_twist_mux',
        default_value='true',
        description='Enable Twist Mux velocity multiplexer'
    )

    # Robot Description
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2a. C++ Controller Node
    controller_cpp_node = Node(
        package='mobile_robot_controller',
        executable='diff_drive_controller_cpp',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            controller_params,
            {'use_sim_time': False, 'publish_joint_states': True}
        ],
        condition=IfCondition(LaunchConfiguration('use_cpp'))
    )

    # 2b. Python Controller Node
    controller_py_node = Node(
        package='mobile_robot_controller',
        executable='diff_drive_controller.py',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            controller_params,
            {'use_sim_time': False, 'publish_joint_states': True}
        ],
        condition=UnlessCondition(LaunchConfiguration('use_cpp'))
    )

    # 3. Serial Hardware Bridge (Talks to Arduino / ESP32)
    serial_bridge_node = Node(
        package='mobile_robot_firmware',
        executable='serial_hardware_bridge.py',
        name='serial_hardware_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate')
        }]
    )

    # 4. Twist Mux Velocity Multiplexer & Joystick Teleop
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg, 'launch', 'joystick_teleop.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false', 'cmd_vel_out': 'cmd_vel_raw'}.items(),
        condition=IfCondition(LaunchConfiguration('use_twist_mux'))
    )

    # 5. Safety Zone Controller (Red Zone stop, Yellow Zone 2x slowdown, auto-resume, RViz toggle button)
    safety_controller_node = Node(
        package='mobile_robot_bringup',
        executable='safety_zone_controller.py',
        name='safety_zone_controller',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 6. Optional RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        use_cpp_arg,
        launch_rviz_arg,
        use_twist_mux_arg,
        robot_state_publisher_node,
        controller_cpp_node,
        controller_py_node,
        serial_bridge_node,
        twist_mux_launch,
        safety_controller_node,
        rviz_node
    ])
