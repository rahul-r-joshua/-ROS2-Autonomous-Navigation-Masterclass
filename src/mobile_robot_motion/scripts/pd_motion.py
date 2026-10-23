#!/usr/bin/env python3
"""
================================================================================
Proportional-Derivative (PD) Local Motion Controller (Angular Heading Steering)
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
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, inverse_matrix, concatenate_matrices


class PDMotion(Node):
    def __init__(self):
        super().__init__("pd_motion_node")

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("kd", 0.1)
        self.declare_parameter("step_size", 0.25)
        self.declare_parameter("max_linear_velocity", 0.30)
        self.declare_parameter("max_angular_velocity", 1.0)

        self.kp = self.get_parameter("kp").value
        self.kd = self.get_parameter("kd").value
        self.step_size = self.get_parameter("step_size").value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value

        # Subscribers and publishers
        self.path_sub = self.create_subscription(Path, "/a_star/path", self.path_callback, 10)
        self.dijkstra_path_sub = self.create_subscription(Path, "/dijkstra/path", self.path_callback, 10)
        self.plan_sub = self.create_subscription(Path, "/plan", self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.next_pose_pub = self.create_publisher(PoseStamped, "/pd/next_pose", 10)

        # Control loop
        self.timer = self.create_timer(0.05, self.control_loop)
        self.global_plan = None

        self.prev_angular_error = 0.0
        self.prev_linear_error = 0.0
        self.last_cycle_time = self.get_clock().now()
        self.get_logger().info("PD Motion Planner node initialized with forward path tracking.")

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

        next_pose: PoseStamped = self.get_next_pose(robot_pose)
        self.next_pose_pub.publish(next_pose)

        # Transform next pose into robot's local base coordinate frame
        robot_tf = quaternion_matrix([
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w,
        ])
        robot_tf[0][3] = robot_pose.pose.position.x
        robot_tf[1][3] = robot_pose.pose.position.y

        next_pose_tf = quaternion_matrix([
            next_pose.pose.orientation.x,
            next_pose.pose.orientation.y,
            next_pose.pose.orientation.z,
            next_pose.pose.orientation.w,
        ])
        next_pose_tf[0][3] = next_pose.pose.position.x
        next_pose_tf[1][3] = next_pose.pose.position.y

        next_pose_robot_tf = concatenate_matrices(inverse_matrix(robot_tf), next_pose_tf)
        target_x = next_pose_robot_tf[0, 3]
        target_y = next_pose_robot_tf[1, 3]
        heading_error = math.atan2(target_y, target_x)

        dt = (self.get_clock().now() - self.last_cycle_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        # If waypoint is substantially behind robot (> 90 deg), rotate in place towards it
        if abs(heading_error) > 1.57:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = max(
                -self.max_angular_velocity,
                min(1.5 * heading_error, self.max_angular_velocity)
            )
            self.cmd_pub.publish(cmd_vel)
            self.prev_angular_error = heading_error
            self.prev_linear_error = 0.0
            self.last_cycle_time = self.get_clock().now()
            return

        # Drive forward while applying PD cross-track steering
        angular_error = target_y
        angular_error_derivative = (angular_error - self.prev_angular_error) / dt
        linear_error = target_x
        linear_error_derivative = (linear_error - self.prev_linear_error) / dt

        cmd_vel = Twist()
        cmd_vel.angular.z = max(
            -self.max_angular_velocity,
            min(self.kp * angular_error + self.kd * angular_error_derivative, self.max_angular_velocity)
        )
        v = self.kp * linear_error + self.kd * linear_error_derivative
        v = max(0.08, min(v, self.max_linear_velocity))
        v *= max(0.1, math.cos(heading_error))
        cmd_vel.linear.x = v

        self.cmd_pub.publish(cmd_vel)
        self.prev_angular_error = angular_error
        self.prev_linear_error = linear_error
        self.last_cycle_time = self.get_clock().now()

    def get_next_pose(self, robot_pose: PoseStamped) -> PoseStamped:
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

        # Search forward along path from closest waypoint for tracking point
        for i in range(closest_idx, len(self.global_plan.poses)):
            pose = self.global_plan.poses[i]
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            if math.hypot(dx, dy) >= self.step_size:
                return pose
        return self.global_plan.poses[-1]


PDMotionPlanner = PDMotion
 
 
def main(args=None):
    rclpy.init(args=args)
    pd_motion = PDMotion()
    rclpy.spin(pd_motion)
    pd_motion.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
