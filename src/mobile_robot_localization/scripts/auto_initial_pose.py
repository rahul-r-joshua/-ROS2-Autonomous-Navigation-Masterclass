#!/usr/bin/env python3
"""
================================================================================
Auto Initial Pose Publisher Node (Zero-Click AMCL Pose Initialization)
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
Publishes initial pose estimate to /initialpose for AMCL particle filter bootstrap.
================================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__('auto_initial_pose')
        
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('cov_x', 0.25)
        self.declare_parameter('cov_y', 0.25)
        self.declare_parameter('cov_yaw', 0.0685)

        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.y = self.get_parameter('y').get_parameter_value().double_value
        self.yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.cov_x = self.get_parameter('cov_x').get_parameter_value().double_value
        self.cov_y = self.get_parameter('cov_y').get_parameter_value().double_value
        self.cov_yaw = self.get_parameter('cov_yaw').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_initial_pose)
        self.publish_count = 0
        self.get_logger().info(f'Auto Initial Pose Node Started: x={self.x}, y={self.y}, yaw={self.yaw}')

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Yaw to Quaternion
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy

        # Covariance matrix (diagonal)
        cov = [0.0] * 36
        cov[0] = self.cov_x      # x
        cov[7] = self.cov_y      # y
        cov[35] = self.cov_yaw   # yaw
        msg.pose.covariance = cov

        self.publisher_.publish(msg)
        self.publish_count += 1
        self.get_logger().info(f'Published initial pose [{self.publish_count}/3] to /initialpose: ({self.x:.2f}, {self.y:.2f})')

        if self.publish_count >= 3:
            self.get_logger().info('Initial pose successfully latched and verified. Shutting down node.')
            self.timer.cancel()
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = AutoInitialPose()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
