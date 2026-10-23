#!/usr/bin/env python3
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def is_simulation_running():
    """Detect if Gazebo Classic or Ignition/Gz Sim is currently running."""
    for proc_pattern in ['gzserver', 'gzclient', 'ign gazebo', 'gz sim', 'ign_gazebo']:
        try:
            res = subprocess.run(['pgrep', '-f', proc_pattern], capture_output=True)
            if res.returncode == 0:
                return True
        except Exception:
            pass
    return False


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mobile_robot_description')
    
    gui_arg_val = LaunchConfiguration('gui').perform(context).lower()
    use_sim_time_arg_val = LaunchConfiguration('use_sim_time').perform(context).lower()
    run_rsp_arg_val = LaunchConfiguration('run_rsp').perform(context).lower()
    model_path = LaunchConfiguration('model').perform(context)
    sim_active = is_simulation_running()

    # 1. Determine use_sim_time FIRST
    if use_sim_time_arg_val in ['true', '1']:
        use_sim_time = True
    elif use_sim_time_arg_val in ['false', '0']:
        use_sim_time = False
    else:  # 'auto'
        use_sim_time = sim_active

    # 2. Determine gui (joint_state_publisher_gui)
    if gui_arg_val in ['true', '1']:
        enable_gui = True
    elif gui_arg_val in ['false', '0']:
        enable_gui = False
    else:  # 'auto'
        enable_gui = not sim_active

    # 3. Determine run_rsp (robot_state_publisher)
    if run_rsp_arg_val in ['true', '1']:
        enable_rsp = True
    elif run_rsp_arg_val in ['false', '0']:
        enable_rsp = False
    else:  # 'auto'
        enable_rsp = not sim_active

    # 4. Determine rviz config:
    # Standalone mode: display.rviz (Fixed Frame: base_footprint)
    # Simulation mode: simulation.rviz (Fixed Frame: odom)
    rviz_config_arg_val = LaunchConfiguration('rviz_config').perform(context)
    if rviz_config_arg_val.lower() in ['auto', 'default', '']:
        if sim_active:
            rviz_config_path = os.path.join(pkg_share, 'rviz', 'simulation.rviz')
        else:
            rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    elif rviz_config_arg_val.lower() in ['display', 'display.rviz', 'standalone']:
        rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    elif rviz_config_arg_val.lower() in ['sim', 'simulation', 'simulation.rviz']:
        rviz_config_path = os.path.join(pkg_share, 'rviz', 'simulation.rviz')
    else:
        rviz_config_path = rviz_config_arg_val

    # Process Xacro
    robot_description = ParameterValue(
        Command(['xacro ', model_path]),
        value_type=str
    )

    nodes_to_launch = []

    # 1. Robot State Publisher (only if needed / not provided by gazebo)
    if enable_rsp:
        nodes_to_launch.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time
                }],
                output='screen'
            )
        )

    # 2. Joint State Publisher GUI or fallback Joint State Publisher
    if enable_gui:
        nodes_to_launch.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description
                }]
            )
        )
    elif not sim_active:
        nodes_to_launch.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description
                }]
            )
        )

    # 3. RViz2 Node
    nodes_to_launch.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }]
        )
    )

    return nodes_to_launch


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_description')
    
    default_model_path = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf.xacro')
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='auto',
        description='Enable joint_state_publisher_gui sliders: auto (true if standalone), true, or false'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='auto',
        description='Use simulation (Gazebo) clock: auto, true, or false'
    )
    
    run_rsp_arg = DeclareLaunchArgument(
        name='run_rsp',
        default_value='auto',
        description='Run robot_state_publisher: auto (false if Gazebo running), true, or false'
    )

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf.xacro file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value='auto',
        description='Path to rviz config file: auto, display, or simulation'
    )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        run_rsp_arg,
        model_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])
