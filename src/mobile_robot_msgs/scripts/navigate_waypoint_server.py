#!/usr/bin/env python3
"""
================================================================================
NavigateWaypoint Action Server Node
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
Simulates navigation towards a goal waypoint with real-time remaining distance feedback.
================================================================================
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from mobile_robot_msgs.action import NavigateWaypoint


class NavigateWaypointServer(Node):
    def __init__(self):
        super().__init__('navigate_waypoint_server')
        self._action_server = ActionServer(
            self,
            NavigateWaypoint,
            '/navigate_waypoint',
            self.execute_callback
        )
        self.get_logger().info('✅ Waypoint Navigation Action Server Ready: Listening on /navigate_waypoint...')

    def execute_callback(self, goal_handle):
        target = goal_handle.request.target_pose.pose.position
        desired_speed = max(0.1, goal_handle.request.desired_linear_speed)
        self.get_logger().info(f'Navigating to waypoint ({target.x:.2f}, {target.y:.2f}) at speed {desired_speed:.2f} m/s')

        # Simulate trajectory from start (0,0) to target
        initial_distance = math.sqrt(target.x**2 + target.y**2)
        if initial_distance == 0.0:
            initial_distance = 5.0

        current_dist = initial_distance
        feedback_msg = NavigateWaypoint.Feedback()
        step = desired_speed * 0.2

        while current_dist > 0.1:
            current_dist = max(0.0, current_dist - step)
            feedback_msg.distance_remaining = float(current_dist)
            feedback_msg.current_linear_speed = float(desired_speed)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.2)

        goal_handle.succeed()
        result = NavigateWaypoint.Result()
        result.success = True
        result.total_distance_traveled = float(initial_distance)
        self.get_logger().info('✅ Reached target waypoint successfully!')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigateWaypointServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
