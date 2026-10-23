#!/usr/bin/env python3
"""
================================================================================
Autonomous Navigation Master Launch File (All-in-One Bringup)
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Automatic World & Map Association:
  - If world is 'nav_challenge' -> Loads nav_challenge.world AND nav_challenge_map.yaml
  - If world is 'maze_world' (or 'maze') -> Loads maze_world.world AND maze_map.yaml
  - If world is 'obstacle_world' (or 'obstacle') -> Loads obstacle_world.world AND obstacle_map.yaml
  - Launches:
      1. Robot Simulation & Controllers (simulate_robot.launch.py)
      2. Map Server & AMCL Localization (global_localization.launch.py)
      3. Nav2 Autonomous Navigation Pipeline (navigation.launch.py)
      4. RViz2 Visualizer (navigation.rviz)
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_world_and_map(context, *args, **kwargs):
    pkg_nav = get_package_share_directory('mobile_robot_navigation')
    pkg_loc = get_package_share_directory('mobile_robot_localization')
    pkg_bringup = get_package_share_directory('mobile_robot_bringup')
    pkg_gazebo = get_package_share_directory('mobile_robot_gazebo')
    pkg_mapping = get_package_share_directory('mobile_robot_mapping')

    raw_world = context.perform_substitution(LaunchConfiguration('world'))
    raw_map = context.perform_substitution(LaunchConfiguration('map'))
    safety_zone = context.perform_substitution(LaunchConfiguration('safety_zone'))
    sim_backend = context.perform_substitution(LaunchConfiguration('sim'))
    params_file = context.perform_substitution(LaunchConfiguration('params_file'))
    use_sim_time = context.perform_substitution(LaunchConfiguration('use_sim_time'))
    autostart = context.perform_substitution(LaunchConfiguration('autostart'))
    use_rviz = context.perform_substitution(LaunchConfiguration('use_rviz'))

    # Resolve world and map paths
    world_clean = raw_world.strip()
    if 'maze' in world_clean.lower():
        world_path = os.path.join(pkg_gazebo, 'worlds', 'maze_world.world')
        default_map_path = os.path.join(pkg_mapping, 'maps', 'maze_map.yaml')
    elif 'obstacle' in world_clean.lower():
        world_path = os.path.join(pkg_gazebo, 'worlds', 'obstacle_world.world')
        default_map_path = os.path.join(pkg_mapping, 'maps', 'obstacle_map.yaml')
    elif 'nav_challenge' in world_clean.lower() or world_clean == '' or world_clean == 'default':
        world_path = os.path.join(pkg_gazebo, 'worlds', 'nav_challenge.world')
        default_map_path = os.path.join(pkg_mapping, 'maps', 'nav_challenge_map.yaml')
    elif os.path.isabs(world_clean) and os.path.exists(world_clean):
        world_path = world_clean
        base_name = os.path.splitext(os.path.basename(world_clean))[0]
        possible_map = os.path.join(pkg_mapping, 'maps', f'{base_name}_map.yaml')
        if not os.path.exists(possible_map):
            possible_map = os.path.join(pkg_mapping, 'maps', f'{base_name}.yaml')
        default_map_path = possible_map if os.path.exists(possible_map) else os.path.join(pkg_mapping, 'maps', 'nav_challenge_map.yaml')
    else:
        world_path = os.path.join(pkg_gazebo, 'worlds', 'nav_challenge.world')
        default_map_path = os.path.join(pkg_mapping, 'maps', 'nav_challenge_map.yaml')

    # If user provided explicit map override, use it
    if raw_map.strip() and raw_map.strip() != 'auto':
        map_path = raw_map.strip()
    else:
        map_path = default_map_path

    if not params_file.strip():
        params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    rviz_config_path = os.path.join(pkg_nav, 'rviz', 'navigation.rviz')

    print(f"\n====================================================================")
    print(f"🚀 [Master Autonomous Navigation Bringup]")
    print(f"   ▶ World File: {world_path}")
    print(f"   ▶ Map YAML:   {map_path}")
    print(f"   ▶ Simulator:  {sim_backend}")
    print(f"   ▶ Safety Zone:{safety_zone}")
    print(f"   ▶ RViz Config:{rviz_config_path}")
    print(f"====================================================================\n")

    actions = []

    # 1. Simulation Bringup (Disable its internal simulation.rviz)
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'simulate_robot.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'safety_zone': safety_zone if safety_zone in ['py', 'cpp'] else 'py',
            'sim': sim_backend,
            'use_sim_time': use_sim_time,
            'use_rviz': 'false'
        }.items()
    )
    actions.append(sim_bringup)

    # If safety_zone is 'none', disable safety zone bypass via dynamic toggle after it spawns
    if safety_zone.lower() == 'none':
        disable_safety_zone = TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '/safety_zone_enable', 'std_msgs/msg/Bool', 'data: false', '--once'],
                    output='log'
                )
            ]
        )
        actions.append(disable_safety_zone)

    # 2. Map Server + AMCL Localization
    localization_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_loc, 'launch', 'global_localization.launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
            'autostart': autostart
        }.items()
    )
    actions.append(localization_bringup)

    # 3. Nav2 Navigation Stack
    navigation_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart
        }.items()
    )
    actions.append(navigation_bringup)

    # 4. Navigation RViz2 Visualizer
    if use_rviz.lower() in ['true', '1', 'yes']:
        nav_rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}]
        )
        actions.append(nav_rviz_node)

    return actions


def generate_launch_description():
    pkg_nav = get_package_share_directory('mobile_robot_navigation')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='nav_challenge',
        description='World alias ("nav_challenge", "maze_world", "obstacle_world") or absolute path'
    )

    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value='auto',
        description='Explicit map YAML path (default "auto" matches selected world)'
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameter YAML file'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup lifecycle nodes'
    )

    declare_safety_zone_arg = DeclareLaunchArgument(
        'safety_zone',
        default_value='py',
        description='Safety zone node language ("py" or "cpp")'
    )

    declare_sim_backend_arg = DeclareLaunchArgument(
        'sim',
        default_value='classic',
        description='Simulation backend: "classic", "ign", or "gz"'
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 Navigation visualizer'
    )

    return LaunchDescription([
        declare_world_arg,
        declare_map_arg,
        declare_params_file_arg,
        declare_use_sim_time_arg,
        declare_autostart_arg,
        declare_safety_zone_arg,
        declare_sim_backend_arg,
        declare_use_rviz_arg,
        OpaqueFunction(function=resolve_world_and_map)
    ])
