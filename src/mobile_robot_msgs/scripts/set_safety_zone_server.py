#!/usr/bin/env python3
"""
================================================================================
SetSafetyZone Service Server Node
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
Service server that receives dynamic safety thresholds for collision avoidance.
================================================================================
"""

import rclpy
from rclpy.node import Node
from mobile_robot_msgs.srv import SetSafetyZone


class SetSafetyZoneServer(Node):
    def __init__(self):
        super().__init__('set_safety_zone_server')
        self.red_zone = 0.5
        self.yellow_zone = 1.0
        self.srv = self.create_service(
            SetSafetyZone,
            '/set_safety_zone',
            self.handle_set_safety_zone
        )
        self.get_logger().info('✅ Safety Zone Service Server Ready: Listening on /set_safety_zone...')

    def handle_set_safety_zone(self, request, response):
        if request.red_zone_threshold <= 0.0 or request.yellow_zone_threshold <= request.red_zone_threshold:
            response.success = False
            response.message = 'Invalid thresholds: red_zone must be > 0 and yellow_zone > red_zone'
            self.get_logger().warn(f'Rejected: {response.message}')
            return response

        self.red_zone = request.red_zone_threshold
        self.yellow_zone = request.yellow_zone_threshold
        response.success = True
        response.message = f'Safety zones updated: Red={self.red_zone:.2f}m, Yellow={self.yellow_zone:.2f}m'
        self.get_logger().info(f'✅ {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SetSafetyZoneServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
