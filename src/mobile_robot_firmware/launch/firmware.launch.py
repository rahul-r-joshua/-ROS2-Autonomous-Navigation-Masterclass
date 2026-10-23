#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino (e.g. /dev/ttyUSB0 or /dev/ttyACM0)'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    hardware_bridge_node = Node(
        package='mobile_robot_firmware',
        executable='serial_hardware_bridge.py',
        name='serial_hardware_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate')
        }]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        hardware_bridge_node
    ])
