#!/usr/bin/env python3
"""
================================================================================
Single Goal Navigation Dispatcher (Nav2 Simple Commander API)
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


# Safe, collision-free destination poses tailored to each simulation world
WORLD_GOALS = {
    'nav_challenge': {'x': 1.0, 'y': 0.5, 'yaw_deg': 0.0},
    'maze_world': {'x': 2.0, 'y': 0.0, 'yaw_deg': 0.0},
    'obstacle_world': {'x': 1.5, 'y': 0.0, 'yaw_deg': 0.0}
}


def get_selected_world():
    world = 'nav_challenge'
    for arg in sys.argv[1:]:
        if arg.startswith('world:='):
            world = arg.split(':=')[1].strip()
        elif arg.startswith('--world='):
            world = arg.split('=')[1].strip()
        elif arg in WORLD_GOALS:
            world = arg
    if world not in WORLD_GOALS:
        for w in WORLD_GOALS:
            if w in world:
                return w
        return 'nav_challenge'
    return world


def main():
    rclpy.init()
    navigator = BasicNavigator()

    world = get_selected_world()
    goal_cfg = WORLD_GOALS[world]

    navigator.waitUntilNav2Active()

    # Define single target goal pose in map frame
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(goal_cfg['x'])
    goal_pose.pose.position.y = float(goal_cfg['y'])
    goal_pose.pose.position.z = 0.0

    yaw_rad = math.radians(goal_cfg.get('yaw_deg', 0.0))
    goal_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    goal_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

    print(f"🚀 [World: {world}] Navigating to Target Pose: (x={goal_cfg['x']}, y={goal_cfg['y']})...")
    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            dx = goal_pose.pose.position.x - feedback.current_pose.pose.position.x
            dy = goal_pose.pose.position.y - feedback.current_pose.pose.position.y
            dist = math.hypot(dx, dy)
            print(f"Distance remaining to goal: {dist:.2f} meters")
        time.sleep(0.5)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("✅ Reached Target Pose Successfully!")
    elif result == TaskResult.CANCELED:
        print("⚠️ Navigation Goal Canceled.")
    elif result == TaskResult.FAILED:
        print("❌ Navigation Goal Failed.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()

