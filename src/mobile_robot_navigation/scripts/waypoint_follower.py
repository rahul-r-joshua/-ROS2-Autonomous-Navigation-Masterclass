#!/usr/bin/env python3
"""
================================================================================
Waypoint Follower Mission Dispatcher (Nav2 Simple Commander API)
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
================================================================================
"""

import sys
import time
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# Safe inspection waypoints tailored to each simulation world
WORLD_WAYPOINTS = {
    'nav_challenge': [
        (1.0, 0.5, 0.0),
        (0.0, 3.0, 90.0),
        (-2.0, 0.0, 180.0),
        (0.0, 0.0, 0.0)
    ],
    'maze_world': [
        (2.0, 0.0, 0.0),
        (2.0, 2.0, 90.0),
        (0.0, 2.0, 180.0),
        (0.0, 0.0, 0.0)
    ],
    'obstacle_world': [
        (1.5, 0.0, 0.0),
        (0.0, 1.0, 90.0),
        (-1.5, 0.0, 180.0),
        (0.0, 0.0, 0.0)
    ]
}


def get_selected_world():
    world = 'nav_challenge'
    for arg in sys.argv[1:]:
        if arg.startswith('world:='):
            world = arg.split(':=')[1].strip()
        elif arg.startswith('--world='):
            world = arg.split('=')[1].strip()
        elif arg in WORLD_WAYPOINTS:
            world = arg
    if world not in WORLD_WAYPOINTS:
        for w in WORLD_WAYPOINTS:
            if w in world:
                return w
        return 'nav_challenge'
    return world


def create_pose(navigator, x, y, yaw_deg=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    world = get_selected_world()
    pts = WORLD_WAYPOINTS[world]

    # Wait for Nav2 to activate
    navigator.waitUntilNav2Active()

    waypoints = [create_pose(navigator, x, y, yaw) for (x, y, yaw) in pts]

    print(f"🚀 [World: {world}] Dispatching Waypoint Patrol Mission ({len(waypoints)} waypoints)...")
    navigator.followWaypoints(waypoints)

    current_wp = -1
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            if feedback.current_waypoint != current_wp:
                if current_wp >= 0:
                    print(f"  ✅ Reached Waypoint {current_wp + 1}/{len(waypoints)}! Transitioning to next...")
                current_wp = feedback.current_waypoint
                target = pts[current_wp]
                print(f"📍 Waypoint {current_wp + 1}/{len(waypoints)}: Navigating to (x={target[0]}, y={target[1]})...")
        time.sleep(0.5)

    if current_wp >= 0:
        print(f"  ✅ Reached Waypoint {current_wp + 1}/{len(waypoints)}!")

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("✅ Waypoint Patrol Mission Completed Successfully!")
    elif result == TaskResult.CANCELED:
        print("⚠️ Waypoint Patrol Mission was Canceled.")
    elif result == TaskResult.FAILED:
        print("❌ Waypoint Patrol Mission Failed.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
