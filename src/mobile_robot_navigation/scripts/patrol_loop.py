#!/usr/bin/env python3
"""
================================================================================
Continuous Autonomous Patrolling Loop (Nav2 Simple Commander API)
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


# Safe perimeter patrol loops tailored to each simulation world
WORLD_PATROL_POINTS = {
    'nav_challenge': [
        (1.0, 0.5, 0.0),
        (0.0, 3.0, 90.0),
        (-2.0, 0.0, 180.0),
        (0.0, -3.0, 270.0),
        (0.0, 0.0, 0.0)
    ],
    'maze_world': [
        (2.0, 0.0, 0.0),
        (2.0, 2.0, 90.0),
        (0.0, 2.0, 180.0),
        (-2.0, 0.0, 270.0),
        (0.0, 0.0, 0.0)
    ],
    'obstacle_world': [
        (1.5, 0.0, 0.0),
        (0.0, 1.0, 90.0),
        (-1.5, 0.0, 180.0),
        (0.0, -1.0, 270.0),
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
        elif arg in WORLD_PATROL_POINTS:
            world = arg
    if world not in WORLD_PATROL_POINTS:
        for w in WORLD_PATROL_POINTS:
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
    pts = WORLD_PATROL_POINTS[world]

    navigator.waitUntilNav2Active()

    patrol_points = [create_pose(navigator, x, y, yaw) for (x, y, yaw) in pts]

    lap = 0
    while rclpy.ok():
        lap += 1
        print(f"\n🔄 [World: {world}] Starting Continuous Patrol Lap #{lap}...")
        for pt_idx, pt in enumerate(patrol_points):
            coords = pts[pt_idx]
            print(f"\n🎯 [Lap #{lap}] Waypoint {pt_idx + 1}/{len(patrol_points)}: Heading to (x={coords[0]}, y={coords[1]})...")
            navigator.goToPose(pt)
            i = 0
            while not navigator.isTaskComplete():
                i += 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    dx = coords[0] - feedback.current_pose.pose.position.x
                    dy = coords[1] - feedback.current_pose.pose.position.y
                    print(f"  Distance remaining: {math.hypot(dx, dy):.2f} m")
                time.sleep(0.5)

            result = navigator.getResult()
            if result != TaskResult.SUCCEEDED:
                print(f"⚠️ Waypoint {pt_idx + 1} reached with status: {result}")
            else:
                print(f"✅ Waypoint {pt_idx + 1} reached successfully!")
            time.sleep(0.5)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

