#!/usr/bin/env python3
"""
================================================================================
Python IMU Republisher Node (Frame Sanitization & EKF Covariance Injection)
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
================================================================================
"""
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Imu

imu_pub = None

def imuCallback(imu):
    global imu_pub
    imu.header.frame_id = "base_footprint_ekf"
    if imu.orientation_covariance[0] == 0.0:
        imu.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
    if imu.angular_velocity_covariance[0] == 0.0:
        imu.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
    if imu.linear_acceleration_covariance[0] == 0.0:
        imu.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]
    imu_pub.publish(imu)


def main():
    global imu_pub
    rclpy.init()
    node = Node("imu_republisher_node")
    time.sleep(1)

    imu_pub = node.create_publisher(Imu, "imu_ekf", 10)
    imu_sub_ = node.create_subscription(Imu, "/imu/data", imuCallback, 10)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
