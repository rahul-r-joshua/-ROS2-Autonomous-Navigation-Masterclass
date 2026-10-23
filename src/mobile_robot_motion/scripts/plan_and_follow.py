#!/usr/bin/env python3
"""
================================================================================
plan_and_follow.py
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
Universal testing and interactive bridge client for ROS 2 / Nav2 Planners
and Controllers. Supports both Mode 1 (Topic bridge) and Mode 2 (Action Server
bridging /compute_path_to_pose and /follow_path).
================================================================================
"""

import sys
import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import rclpy.time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, FollowPath


def normalize_planner_name(name: str) -> str:
    n = name.strip().lower().replace("_", "")
    if "dijkstra" in n:
        return "Dijkstra"
    if "astar" in n or "grid" in n:
        return "GridBased"
    return name


def normalize_controller_name(name: str) -> str:
    n = name.strip().lower().replace("_", "")
    if "pure" in n or "pursuit" in n:
        return "PurePursuit"
    if "pd" in n or "motion" in n:
        return "PDMotion"
    return name


class PlanAndFollowClient(Node):
    def __init__(self, planner_id: str, controller_id: str):
        super().__init__("plan_and_follow_client")
        self.planner_id = normalize_planner_name(planner_id)
        self.controller_id = normalize_controller_name(controller_id)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action clients for C++ Nav2 Planner & Controller Servers
        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.follow_client = ActionClient(self, FollowPath, "/follow_path")

        # Standalone topic publishers / subscribers for Mode 1
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.rviz_goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.on_rviz_goal_received, 10
        )

        self.is_navigating = False
        self.is_interactive = False
        self.current_goal_handle = None

        self.get_logger().info(
            f"Plan and Follow Client initialized with Planner [{self.planner_id}] "
            f"and Controller [{self.controller_id}]."
        )

    def wait_for_future(self, future, timeout_sec=None):
        if self.is_interactive:
            # When running interactively in rclpy.spin(), wait asynchronously without blocking executor
            start = time.time()
            while not future.done() and rclpy.ok():
                if timeout_sec and (time.time() - start) > timeout_sec:
                    return False
                time.sleep(0.02)
            return future.done()
        else:
            # Standalone one-shot CLI mode, no background spin()
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            return future.done()

    def on_rviz_goal_received(self, goal_msg: PoseStamped):
        if self.is_navigating:
            self.get_logger().info("New 2D Goal Pose received while navigating: preempting active path...")
            if self.current_goal_handle:
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception:
                    pass

        self.get_logger().info(
            f"Received interactive goal at ({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}). Planning..."
        )
        # Execute mission in separate daemon thread to prevent deadlocking rclpy.spin()!
        threading.Thread(target=self.execute_mission, args=(goal_msg,), daemon=True).start()

    def execute_mission(self, goal_pose: PoseStamped):
        self.is_navigating = True

        # Check if C++ Nav2 Action Server is available
        if self.plan_client.wait_for_server(timeout_sec=2.0) and self.follow_client.wait_for_server(timeout_sec=2.0):
            self.execute_nav2_actions(goal_pose)
        else:
            # Standalone Topic Mode (Mode 1)
            self.get_logger().info("Nav2 action servers not detected. Using direct topic publication mode.")
            self.goal_pub.publish(goal_pose)
            self.is_navigating = False

    def execute_nav2_actions(self, goal_pose: PoseStamped):
        # 1. Compute Path Action Goal
        plan_goal = ComputePathToPose.Goal()
        plan_goal.planner_id = self.planner_id
        plan_goal.goal = goal_pose

        self.get_logger().info(f"Requesting global plan from [{self.planner_id}]...")
        send_goal_future = self.plan_client.send_goal_async(plan_goal)
        self.wait_for_future(send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("ComputePathToPose goal was rejected by planner_server!")
            self.is_navigating = False
            return

        result_future = goal_handle.get_result_async()
        self.wait_for_future(result_future)
        result = result_future.result().result

        if not result or not result.path.poses:
            self.get_logger().error("Planner failed to return a valid path!")
            self.is_navigating = False
            return

        path = result.path
        self.get_logger().info(f"Path computed successfully with {len(path.poses)} waypoints.")

        # 2. Follow Path Action Goal
        follow_goal = FollowPath.Goal()
        follow_goal.controller_id = self.controller_id
        follow_goal.path = path

        self.get_logger().info(f"Streaming trajectory to controller [{self.controller_id}]...")
        send_follow_future = self.follow_client.send_goal_async(follow_goal)
        self.wait_for_future(send_follow_future)

        follow_handle = send_follow_future.result()
        if not follow_handle or not follow_handle.accepted:
            self.get_logger().error("FollowPath goal was rejected by controller_server!")
            self.is_navigating = False
            return

        self.current_goal_handle = follow_handle
        get_follow_result_future = follow_handle.get_result_async()
        self.wait_for_future(get_follow_result_future)

        follow_res = get_follow_result_future.result()
        if follow_res:
            self.get_logger().info(f"Controller finished with status code: {follow_res.status} (SUCCEEDED = 4)")
        self.is_navigating = False
        self.current_goal_handle = None


def main():
    rclpy.init()

    planner = "GridBased"
    controller = "PurePursuit"
    target_x = None
    target_y = None

    if len(sys.argv) > 1:
        planner = sys.argv[1]
    if len(sys.argv) > 2:
        controller = sys.argv[2]
    if len(sys.argv) > 4:
        target_x = float(sys.argv[3])
        target_y = float(sys.argv[4])

    client = PlanAndFollowClient(planner, controller)

    if target_x is not None and target_y is not None:
        # One-shot coordinate mode
        client.is_interactive = False
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = target_x
        goal_msg.pose.position.y = target_y
        goal_msg.pose.orientation.w = 1.0
        client.execute_mission(goal_msg)
    else:
        # Interactive 2D Goal Pose mode
        client.is_interactive = True
        print("\n=======================================================")
        print("  Interactive 2D Goal Pose Bridge Active!")
        print(f"  Planner:    {client.planner_id}")
        print(f"  Controller: {client.controller_id}")
        print("  Click and drag '2D Goal Pose' in RViz2 to navigate...")
        print("=======================================================\n")
        try:
            rclpy.spin(client)
        except KeyboardInterrupt:
            pass

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
