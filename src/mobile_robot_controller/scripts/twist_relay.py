#!/usr/bin/env python3
"""
================================================================================
Twist Relay Node
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Relays velocity commands between stamped and unstamped formats:
 1. /input_joy/cmd_vel_stamped (TwistStamped from joy_teleop) -> /joy_vel (Twist for twist_mux)
 2. /input_joy/cmd_vel -> /joy_vel
 3. /cmd_vel_raw (Twist) -> /cmd_vel_stamped (TwistStamped)
================================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelayNode(Node):
    def __init__(self):
        super().__init__("twist_relay")

        # Subscribes to joy_teleop output (stamped) and publishes unstamped for twist_mux
        self.joy_sub = self.create_subscription(
            TwistStamped,
            "/input_joy/cmd_vel_stamped",
            self.joy_twist_callback,
            10
        )
        self.joy_pub = self.create_publisher(
            Twist,
            "/joy_vel",
            10
        )

        # Also support /input_joy/cmd_vel (unstamped input)
        self.joy_unstamped_sub = self.create_subscription(
            Twist,
            "/input_joy/cmd_vel",
            self.joy_unstamped_callback,
            10
        )

        # Controller / cmd_vel relay
        self.controller_sub = self.create_subscription(
            Twist,
            "/cmd_vel_raw",
            self.controller_twist_callback,
            10
        )
        self.controller_pub = self.create_publisher(
            TwistStamped,
            "/cmd_vel_stamped",
            10
        )

        self.get_logger().info("Twist Relay Node Initialized.")

    def joy_twist_callback(self, msg: TwistStamped):
        twist = Twist()
        twist = msg.twist
        self.joy_pub.publish(twist)

    def joy_unstamped_callback(self, msg: Twist):
        self.joy_pub.publish(msg)

    def controller_twist_callback(self, msg: Twist):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg
        self.controller_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
