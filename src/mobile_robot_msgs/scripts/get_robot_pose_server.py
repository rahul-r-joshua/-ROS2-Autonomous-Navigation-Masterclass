#!/usr/bin/env python3
"""
================================================================================
GetRobotPose Service Server Node
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
Queries the live TF tree to return current robot pose and yaw heading.
================================================================================
"""

import math
import rclpy
from rclpy.node import Node
import tf2_ros
from mobile_robot_msgs.srv import GetRobotPose


def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class GetRobotPoseServer(Node):
    def __init__(self):
        super().__init__('get_robot_pose_server')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.srv = self.create_service(
            GetRobotPose,
            '/get_robot_pose',
            self.handle_get_robot_pose
        )
        self.get_logger().info('✅ Robot Pose Service Server Ready: Listening on /get_robot_pose...')

    def handle_get_robot_pose(self, request, response):
        frame = request.target_frame if request.target_frame else 'odom'
        try:
            transform = self.tf_buffer.lookup_transform(
                frame,
                'base_footprint',
                rclpy.time.Time()
            )
            response.pose.header = transform.header
            response.pose.pose.position.x = transform.transform.translation.x
            response.pose.pose.position.y = transform.transform.translation.y
            response.pose.pose.position.z = transform.transform.translation.z
            response.pose.pose.orientation = transform.transform.rotation

            rot = transform.transform.rotation
            response.heading_yaw = float(euler_from_quaternion(rot.x, rot.y, rot.z, rot.w))
            response.success = True
            self.get_logger().info(f'Pose in [{frame}]: ({response.pose.pose.position.x:.2f}, {response.pose.pose.position.y:.2f}), yaw={response.heading_yaw:.2f} rad')
        except Exception as ex:
            response.success = False
            self.get_logger().warn(f'Could not lookup robot pose in [{frame}]: {str(ex)}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = GetRobotPoseServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
