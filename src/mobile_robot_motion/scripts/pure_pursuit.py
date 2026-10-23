#!/usr/bin/env python3
"""
================================================================================
Pure Pursuit Local Path Tracking Controller (Geometric Lookahead Control)
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
================================================================================
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, inverse_matrix, concatenate_matrices


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit_motion_planner_node")

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters (Fast, responsive defaults)
        self.declare_parameter("look_ahead_distance", 0.40)
        self.declare_parameter("max_linear_velocity", 0.35)
        self.declare_parameter("max_angular_velocity", 1.80)

        self.look_ahead_distance = self.get_parameter("look_ahead_distance").value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value

        # Subscribers and publishers
        self.path_sub = self.create_subscription(Path, "/a_star/path", self.path_callback, 10)
        self.dijkstra_path_sub = self.create_subscription(Path, "/dijkstra/path", self.path_callback, 10)
        self.plan_sub = self.create_subscription(Path, "/plan", self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.carrot_pose_pub = self.create_publisher(PoseStamped, "/pure_pursuit/carrot", 10)

        # Control loop
        self.timer = self.create_timer(0.05, self.control_loop)
        self.global_plan = None
        self.get_logger().info("Pure Pursuit node initialized with fast responsive turning.")

    def path_callback(self, path: Path):
        self.global_plan = path
        self.get_logger().info(f"Received path with {len(path.poses)} poses in frame [{path.header.frame_id}].")

    def control_loop(self):
        if not self.global_plan or not self.global_plan.poses:
            return

        plan_frame = self.global_plan.header.frame_id if self.global_plan.header.frame_id else "map"

        # Lookup robot's current pose in the plan frame
        try:
            robot_pose_transform = self.tf_buffer.lookup_transform(
                plan_frame, "base_footprint", rclpy.time.Time()
            )
        except Exception as ex:
            self.get_logger().warn(f"Could not transform [{plan_frame}] to [base_footprint]: {ex}")
            return

        robot_pose = PoseStamped()
        robot_pose.header.frame_id = plan_frame
        robot_pose.pose.position.x = robot_pose_transform.transform.translation.x
        robot_pose.pose.position.y = robot_pose_transform.transform.translation.y
        robot_pose.pose.orientation = robot_pose_transform.transform.rotation

        # Check goal arrival
        goal_pose = self.global_plan.poses[-1]
        dist_to_goal = math.hypot(
            goal_pose.pose.position.x - robot_pose.pose.position.x,
            goal_pose.pose.position.y - robot_pose.pose.position.y
        )

        if dist_to_goal <= 0.20:
            gq = goal_pose.pose.orientation
            gq_norm = math.hypot(gq.z, gq.w)
            if gq_norm > 0.01:
                # 1. Extract robot yaw from quaternion
                rq = robot_pose.pose.orientation
                robot_yaw = math.atan2(2.0 * (rq.w * rq.z + rq.x * rq.y), 1.0 - 2.0 * (rq.y * rq.y + rq.z * rq.z))

                # 2. Extract goal yaw from quaternion
                goal_yaw = math.atan2(2.0 * (gq.w * gq.z + gq.x * gq.y), 1.0 - 2.0 * (gq.y * gq.y + gq.z * gq.z))

                # 3. Compute normalized yaw error (-pi to pi)
                yaw_error = math.atan2(math.sin(goal_yaw - robot_yaw), math.cos(goal_yaw - robot_yaw))

                # 4. In-place rotation until aligned within 0.05 rad (~2.8 degrees)
                if abs(yaw_error) > 0.05:
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0
                    rot_speed = 2.0 * yaw_error
                    if abs(rot_speed) < 0.45:
                        rot_speed = 0.45 if yaw_error > 0.0 else -0.45
                    cmd_vel.angular.z = max(-self.max_angular_velocity, min(rot_speed, self.max_angular_velocity))
                    self.cmd_pub.publish(cmd_vel)
                    return

            self.get_logger().info("🎉 Goal Position and Orientation Reached!")
            self.global_plan = None
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            return

        carrot_pose: PoseStamped = self.get_carrot_pose(robot_pose)
        self.carrot_pose_pub.publish(carrot_pose)

        # Transform carrot pose into robot's local base coordinate frame
        robot_tf = quaternion_matrix([
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w,
        ])
        robot_tf[0][3] = robot_pose.pose.position.x
        robot_tf[1][3] = robot_pose.pose.position.y

        carrot_pose_tf = quaternion_matrix([
            carrot_pose.pose.orientation.x,
            carrot_pose.pose.orientation.y,
            carrot_pose.pose.orientation.z,
            carrot_pose.pose.orientation.w,
        ])
        carrot_pose_tf[0][3] = carrot_pose.pose.position.x
        carrot_pose_tf[1][3] = carrot_pose.pose.position.y

        carrot_pose_robot_tf = concatenate_matrices(inverse_matrix(robot_tf), carrot_pose_tf)
        target_x = carrot_pose_robot_tf[0, 3]
        target_y = carrot_pose_robot_tf[1, 3]
        heading_error = math.atan2(target_y, target_x)

        # 1. Fast, responsive turn-in-place when heading offset is significant (> 35 deg / 0.60 rad)
        if abs(heading_error) > 0.60:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.05  # Slight forward creep maintains steady odometry progress
            rot_speed = 2.5 * heading_error
            if abs(rot_speed) < 0.60:
                rot_speed = 0.60 if heading_error > 0.0 else -0.60
            cmd_vel.angular.z = max(
                -self.max_angular_velocity,
                min(rot_speed, self.max_angular_velocity)
            )
            self.cmd_pub.publish(cmd_vel)
            return

        # 2. Well-aligned with path: Drive forward with responsive Pure Pursuit curvature steering
        curvature = self.get_curvature(target_x, target_y)
        cmd_vel = Twist()
        linear_v = min(self.max_linear_velocity, max(0.12, dist_to_goal))
        cmd_vel.linear.x = linear_v
        steer_rate = curvature * linear_v * 1.5
        cmd_vel.angular.z = max(
            -self.max_angular_velocity,
            min(steer_rate, self.max_angular_velocity)
        )
        self.cmd_pub.publish(cmd_vel)

    def get_carrot_pose(self, robot_pose: PoseStamped) -> PoseStamped:
        # Find closest waypoint along the plan
        closest_idx = 0
        min_dist_sq = float('inf')
        for i, pose in enumerate(self.global_plan.poses):
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            dist_sq = dx * dx + dy * dy
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_idx = i

        # Search forward along path from closest waypoint for lookahead point
        for i in range(closest_idx, len(self.global_plan.poses)):
            pose = self.global_plan.poses[i]
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            if math.hypot(dx, dy) >= self.look_ahead_distance:
                return pose
        return self.global_plan.poses[-1]

    def get_curvature(self, target_x: float, target_y: float) -> float:
        dist_sq = target_x * target_x + target_y * target_y
        if dist_sq > 0.001:
            return 2.0 * target_y / dist_sq
        return 0.0


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
