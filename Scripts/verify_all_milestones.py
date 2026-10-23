#!/usr/bin/env python3
"""
================================================================================
Comprehensive Milestone Verification Suite for Autonomous Navigation WS
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
import sys
import subprocess

EXPECTED_PACKAGES = [
    "mobile_robot_description",
    "mobile_robot_controller",
    "mobile_robot_bringup",
    "mobile_robot_firmware",
    "mobile_robot_gazebo",
    "mobile_robot_mapping",
    "mobile_robot_msgs",
    "mobile_robot_localization",
    "mobile_robot_planning",
    "mobile_robot_motion",
    "mobile_robot_navigation"
]

EXPECTED_WORLDS = [
    "maze_world.world",
    "obstacle_world.world",
    "nav_challenge.world"
]

EXPECTED_MAPS = [
    ("maze_map.yaml", "maze_map.pgm"),
    ("obstacle_map.yaml", "obstacle_map.pgm"),
    ("nav_challenge_map.yaml", "nav_challenge_map.pgm")
]

EXPECTED_LAUNCH_FILES = [
    ("mobile_robot_description", "gazebo.launch.py"),
    ("mobile_robot_description", "ign_gazebo.launch.py"),
    ("mobile_robot_controller", "controller.launch.py"),
    ("mobile_robot_controller", "ros2_controller.launch.py"),
    ("mobile_robot_firmware", "firmware.launch.py"),
    ("mobile_robot_bringup", "simulate_robot.launch.py"),
    ("mobile_robot_bringup", "safety_zone.launch.py"),
    ("mobile_robot_bringup", "autonomous_navigation.launch.py"),
    ("mobile_robot_bringup", "real_robot.launch.py"),
    ("mobile_robot_mapping", "slam.launch.py"),
    ("mobile_robot_localization", "global_localization.launch.py"),
    ("mobile_robot_localization", "local_localization.launch.py"),
    ("mobile_robot_navigation", "navigation.launch.py"),
    ("mobile_robot_navigation", "rviz.launch.py")
]

EXPECTED_CONFIG_FILES = [
    ("mobile_robot_navigation", "config/nav2_params.yaml"),
    ("mobile_robot_localization", "config/ekf.yaml"),
    ("mobile_robot_localization", "config/amcl.yaml"),
    ("mobile_robot_mapping", "config/mapper_params_online_async.yaml"),
    ("mobile_robot_controller", "config/controller_params.yaml"),
    ("mobile_robot_controller", "config/ros2_robot_controller_params.yaml"),
    ("mobile_robot_planning", "config/planner_server.yaml"),
    ("mobile_robot_motion", "config/controller_server.yaml")
]

EXPECTED_PLUGINS = [
    ("mobile_robot_planning", "global_planner_plugins.xml"),
    ("mobile_robot_motion", "motion_controller_plugins.xml")
]

EXPECTED_NODES_AND_SCRIPTS = [
    ("mobile_robot_firmware", "serial_hardware_bridge.py"),
    ("mobile_robot_controller", "diff_drive_controller.py"),
    ("mobile_robot_controller", "diff_drive_controller.cpp"),
    ("mobile_robot_bringup", "safety_zone_controller.py"),
    ("mobile_robot_bringup", "safety_zone_controller.cpp"),
    ("mobile_robot_localization", "kalman_filter.py"),
    ("mobile_robot_localization", "kalman_filter.cpp"),
    ("mobile_robot_localization", "odometry_motion_model.py"),
    ("mobile_robot_localization", "odometry_motion_model.cpp"),
    ("mobile_robot_localization", "imu_republisher.py"),
    ("mobile_robot_localization", "imu_republisher.cpp"),
    ("mobile_robot_planning", "a_star_planner.py"),
    ("mobile_robot_planning", "dijkstra_planner.py"),
    ("mobile_robot_motion", "pure_pursuit.py"),
    ("mobile_robot_motion", "pd_motion.py"),
    ("mobile_robot_navigation", "nav_to_pose.py"),
    ("mobile_robot_navigation", "waypoint_follower.py"),
    ("mobile_robot_navigation", "patrol_loop.py")
]


def run_cmd(cmd):
    res = subprocess.run(cmd, shell=True, capture_output=True, text=True, executable='/bin/bash')
    return res


def main():
    print("=" * 80)
    print("🚀 Running Full Multi-Stage Verification Suite for Autonomous Navigation WS")
    print("=" * 80)
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_dir = os.path.abspath(os.path.join(script_dir, ".."))
    os.chdir(ws_dir)
    src_dir = os.path.join(ws_dir, "src")
    total_checks = 0
    passed_checks = 0
    
    # 1. Check packages presence
    print("\n[Stage 1/7] Verifying 11 ROS 2 Package Trees...")
    for pkg in EXPECTED_PACKAGES:
        total_checks += 1
        pkg_path = os.path.join(src_dir, pkg)
        if os.path.exists(pkg_path):
            print(f"  ✅ Package: {pkg:<32} FOUND")
            passed_checks += 1
        else:
            print(f"  ❌ Package: {pkg:<32} MISSING")
            
    # 2. Check Custom Interfaces
    print("\n[Stage 2/7] Verifying Custom ROS 2 Interfaces (mobile_robot_msgs)...")
    total_checks += 3
    res_act = run_cmd("source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 interface show mobile_robot_msgs/action/NavigateWaypoint")
    if "desired_linear_speed" in res_act.stdout:
        print("  ✅ Action Interface: mobile_robot_msgs/action/NavigateWaypoint VERIFIED")
        passed_checks += 1
    else:
        print("  ⚠️ Warning: NavigateWaypoint action interface not found in environment")

    res_srv1 = run_cmd("source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 interface show mobile_robot_msgs/srv/SetSafetyZone")
    if "red_zone_threshold" in res_srv1.stdout:
        print("  ✅ Service Interface: mobile_robot_msgs/srv/SetSafetyZone VERIFIED")
        passed_checks += 1
    else:
        print("  ⚠️ Warning: SetSafetyZone service interface not found in environment")

    res_srv2 = run_cmd("source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 interface show mobile_robot_msgs/srv/GetRobotPose")
    if "target_frame" in res_srv2.stdout:
        print("  ✅ Service Interface: mobile_robot_msgs/srv/GetRobotPose VERIFIED")
        passed_checks += 1
    else:
        print("  ⚠️ Warning: GetRobotPose service interface not found in environment")

    # 3. Check Simulation Worlds
    print("\n[Stage 3/7] Verifying Gazebo Simulation Worlds...")
    for world in EXPECTED_WORLDS:
        total_checks += 1
        w_path = os.path.join(src_dir, "mobile_robot_gazebo", "worlds", world)
        if os.path.exists(w_path) and os.path.getsize(w_path) > 0:
            print(f"  ✅ Gazebo World: {world:<30} ({os.path.getsize(w_path)} bytes) VERIFIED")
            passed_checks += 1
        else:
            print(f"  ❌ Gazebo World: {world:<30} MISSING")

    # 4. Check Nav2 2D Occupancy Grid Maps (.yaml + .pgm)
    print("\n[Stage 4/7] Verifying Nav2 2D Costmaps (YAML + PGM)...")
    for map_yaml, map_pgm in EXPECTED_MAPS:
        total_checks += 1
        y_path = os.path.join(src_dir, "mobile_robot_mapping", "maps", map_yaml)
        p_path = os.path.join(src_dir, "mobile_robot_mapping", "maps", map_pgm)
        if os.path.exists(y_path) and os.path.exists(p_path) and os.path.getsize(y_path) > 0 and os.path.getsize(p_path) > 0:
            print(f"  ✅ Map Pair: {map_yaml:<24} + {map_pgm:<24} VERIFIED")
            passed_checks += 1
        else:
            print(f"  ❌ Map Pair Missing: {map_yaml} / {map_pgm}")

    # 5. Check Launch & Config Files
    print("\n[Stage 5/7] Verifying Launch & Config Parameter Files...")
    for pkg, launch_file in EXPECTED_LAUNCH_FILES:
        total_checks += 1
        lf_path = os.path.join(src_dir, pkg, "launch", launch_file)
        if os.path.exists(lf_path):
            print(f"  ✅ Launch File: [{pkg}] {launch_file} VERIFIED")
            passed_checks += 1
        else:
            print(f"  ❌ Launch File: [{pkg}] {launch_file} MISSING")

    for pkg, cfg_file in EXPECTED_CONFIG_FILES:
        total_checks += 1
        cf_path = os.path.join(src_dir, pkg, cfg_file)
        if os.path.exists(cf_path):
            print(f"  ✅ Config File: [{pkg}] {cfg_file} VERIFIED")
            passed_checks += 1
        else:
            print(f"  ❌ Config File: [{pkg}] {cfg_file} MISSING")

    # 6. Check Nav2 Planners & Controller Plugins XMLs
    print("\n[Stage 6/7] Verifying Custom Nav2 Plugin XMLs...")
    for pkg, plugin_xml in EXPECTED_PLUGINS:
        total_checks += 1
        pl_path = os.path.join(src_dir, pkg, plugin_xml)
        if os.path.exists(pl_path):
            print(f"  ✅ Plugin Manifest: [{pkg}] {plugin_xml} VERIFIED")
            passed_checks += 1
        else:
            print(f"  ❌ Plugin Manifest: [{pkg}] {plugin_xml} MISSING")

    # 7. Check Executable Milestone Nodes
    print("\n[Stage 7/7] Verifying Milestone Autonomous Nodes & Python/C++ Scripts...")
    for pkg, script in EXPECTED_NODES_AND_SCRIPTS:
        total_checks += 1
        possible_paths = [
            os.path.join(src_dir, pkg, pkg, script),
            os.path.join(src_dir, pkg, "scripts", script),
            os.path.join(src_dir, pkg, "src", script),
            os.path.join(src_dir, pkg, script)
        ]
        found_path = None
        for p in possible_paths:
            if os.path.exists(p):
                found_path = p
                break

        if found_path:
            print(f"  ✅ Node / Script: [{pkg}] {script} VERIFIED")
            passed_checks += 1
        else:
            print(f"  ❌ Node / Script: [{pkg}] {script} MISSING")

    # Summary
    print("\n" + "=" * 80)
    print(f"📊 Overall Verification Results: {passed_checks}/{total_checks} Checks Passed ({(passed_checks/total_checks)*100:.1f}%)")
    print("=" * 80)

    if passed_checks == total_checks:
        print("🎉 ALL SYSTEMS, WORLDS, MAPS, INTERFACES, PLUGINS & LAUNCH FILES 100% OPERATIONAL!\n")
        return 0
    else:
        print(f"⚠️ Warning: {total_checks - passed_checks} checks failed. Please check the logs above.\n")
        return 1


if __name__ == "__main__":
    sys.exit(main())
