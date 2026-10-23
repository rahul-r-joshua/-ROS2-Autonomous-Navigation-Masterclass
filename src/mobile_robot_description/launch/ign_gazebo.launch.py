#!/usr/bin/env python3
"""
================================================================================
Ignition Gazebo (ign) & Modern Gazebo (gz) Simulation Launch File
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Launches Ignition Gazebo / Modern Gz Sim with:
 1. Robot State Publisher (URDF/Xacro)
 2. Gazebo Sim Server & GUI (via ros_gz_sim)
 3. Entity Spawner (via ros_gz_sim create)
 4. ROS-GZ Bridge (bridges cmd_vel, odom, tf, scan, camera, imu, clock)
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('mobile_robot_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # Ensure Wayland compatibility on GNOME/Ubuntu
    set_qt_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'mobile_robot.urdf.xacro')

    gz_args_arg = DeclareLaunchArgument(
        'gz_args',
        default_value='-r empty.sdf',
        description='Arguments to be passed to Gazebo Sim / Ignition'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Path to world SDF file to load in Ignition/Gz Sim'
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Enable ros2_control plugin (true) or classic diff_drive plugin (false)'
    )

    # 1. Robot Description from Xacro (with is_ignition:=true)
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' is_ignition:=true',
            ' use_ros2_control:=', LaunchConfiguration('use_ros2_control')
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    default_empty_sdf = os.path.join(desc_pkg, 'worlds', 'empty.sdf')

    # 2. Gazebo Sim / Ignition launcher (loads world SDF if provided, else default empty.sdf with sensors)
    gz_args_cmd = PythonExpression([
        "'-r ' + '", LaunchConfiguration('world'), "' if '", LaunchConfiguration('world'), "' != '' else '-r " + default_empty_sdf + "'"
    ])

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args_cmd}.items()
    )

    # 3. Entity Spawner
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'four_wheel_robot',
            '-z', '0.05'
        ]
    )

    # 4. ROS-GZ Parameter Bridge (Twist, Scan, Camera, CameraInfo, IMU, Clock)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}]
    )

    # 4B. Joint State and Odometry Bridge (Only active when NOT using ros2_control to prevent TF timestamp conflicts)
    joint_states_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(LaunchConfiguration('use_ros2_control'))
    )

    return LaunchDescription([
        set_qt_platform,
        gz_args_arg,
        world_arg,
        use_ros2_control_arg,
        robot_state_publisher_node,
        gz_sim_launch,
        spawn_node,
        bridge_node,
        joint_states_bridge_node,
    ])
