#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_desc = get_package_share_directory('mobile_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    default_model_path = os.path.join(pkg_desc, 'urdf', 'mobile_robot.urdf.xacro')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf.xacro file'
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='false',
        description='Enable ros2_control plugin (true) or classic diff_drive plugin (false)'
    )

    default_world = os.path.join(pkg_desc, 'worlds', 'empty.world')

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Full path to Gazebo world file'
    )

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set to false to run gzserver only in headless mode'
    )

    # Robot Description
    robot_description = ParameterValue(
        Command([
            'xacro ', LaunchConfiguration('model'),
            ' is_ignition:=false',
            ' use_ros2_control:=', LaunchConfiguration('use_ros2_control')
        ]),
        value_type=str
    )

    # 1. Robot State Publisher (with use_sim_time=True for Gazebo)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 2. Gazebo Server & Client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui')
        }.items()
    )

    # 3. Spawn Robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'four_wheel_robot',
            '-z', '0.05'
        ],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        use_ros2_control_arg,
        world_arg,
        gui_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_entity
    ])
