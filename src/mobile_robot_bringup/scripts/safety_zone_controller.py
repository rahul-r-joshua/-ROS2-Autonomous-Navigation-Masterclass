#!/usr/bin/env python3
"""
================================================================================
Mobile Robot Safety Zone Controller (LiDAR Collision Avoidance)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Safety Rules:
 1. Red Zone (1x Robot Size ~ 0.45m):
    - Robot stops completely when an obstacle is within 0.45m of the robot footprint center.
    - Prevents colliding with forward obstacles; allows reversing away.
 2. Yellow Zone (2x Robot Size ~ 0.90m):
    - Robot slows down by 2x (0.5x linear speed) when obstacle is between 0.45m and 0.90m.
 3. Green Zone (> 0.90m):
    - Full normal speed (1.0x).
    - Automatically restores full speed as soon as obstacle moves away!
 4. Clean Steady RViz Display:
    - Displays clean Red (0.45m) and Yellow (0.90m) zone circles matching laser points.
================================================================================
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class SafetyZoneController(Node):
    def __init__(self):
        super().__init__('safety_zone_controller')

        # ----------------------------------------------------------------------
        # 1. Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('red_zone_distance', 0.45)     # 1x robot size: Stop
        self.declare_parameter('yellow_zone_distance', 0.90)  # 2x robot size: 2x slowdown
        self.declare_parameter('fov_angle_deg', 90.0)         # Forward detection (+/- 45 deg)
        self.declare_parameter('slowdown_factor', 0.5)
        self.declare_parameter('lidar_x_offset', 0.10)        # LiDAR mounting offset in base frame

        self.red_dist = self.get_parameter('red_zone_distance').get_parameter_value().double_value
        self.yellow_dist = self.get_parameter('yellow_zone_distance').get_parameter_value().double_value
        self.fov_rad = math.radians(self.get_parameter('fov_angle_deg').get_parameter_value().double_value)
        self.slowdown = self.get_parameter('slowdown_factor').get_parameter_value().double_value
        self.lidar_x_offset = self.get_parameter('lidar_x_offset').get_parameter_value().double_value

        # State
        self.min_obstacle_distance = float('inf')
        self.zone_state = "UNKNOWN"  # "UNKNOWN", "GREEN", "YELLOW", "RED"
        self.raw_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.manual_safety_active = False
        self.has_fresh_raw_cmd = False
        self.safety_zone_enabled = True

        # Raw command from Twist Mux or Joystick
        self.sub_cmd_raw = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_raw_callback,
            10
        )
        self.sub_joy = self.create_subscription(
            Twist,
            '/joy_vel',
            self.cmd_raw_callback,
            10
        )

        # Safety stop subscription (true: activate safety, false: deactivate safety)
        self.sub_safety_stop = self.create_subscription(
            Bool,
            '/safety_stop',
            self.safety_stop_callback,
            10
        )

        # Safety zone dynamic enable/disable toggle
        self.sub_safety_enable = self.create_subscription(
            Bool,
            '/safety_zone_enable',
            self.safety_enable_callback,
            10
        )

        # LiDAR scan (SensorDataQoS for compatibility with both Gazebo and physical hardware)
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Filtered safe command to Diff Drive Controller
        self.pub_cmd_safe = self.create_publisher(Twist, '/cmd_vel', 10)

        # Clean zone boundary circles for RViz
        self.pub_markers = self.create_publisher(MarkerArray, '/safety_zone_markers', 10)

        # Control loop at 20 Hz
        self.timer = self.create_timer(1.0 / 20.0, self.control_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🛡️  Safety Zone Controller Initialized')
        self.get_logger().info(f'   🔴 Red Zone (1x Robot Size):    {self.red_dist:.2f} m  -> FULL STOP')
        self.get_logger().info(f'   🟡 Yellow Zone (2x Robot Size): {self.yellow_dist:.2f} m  -> 2x SLOWDOWN')
        self.get_logger().info(f'   🟢 Green Zone:                  > {self.yellow_dist:.2f} m -> FULL SPEED (Auto-Resume)')
        self.get_logger().info('=' * 60)

    def safety_enable_callback(self, msg: Bool):
        if msg.data != self.safety_zone_enabled:
            self.safety_zone_enabled = msg.data
            if self.safety_zone_enabled:
                self.get_logger().info('🛡️  Safety Zone ENABLED: Collision avoidance active.')
            else:
                self.get_logger().warn('⚠️  Safety Zone DISABLED: Bypass mode active (100% speed pass-through).')

    def scan_callback(self, msg: LaserScan):
        """Measures closest obstacle in forward field of view relative to robot base."""
        half_fov = self.fov_rad / 2.0
        valid_distances = []
        angle = msg.angle_min

        for r in msg.ranges:
            if math.isfinite(r) and not math.isnan(r) and not math.isinf(r) and (msg.range_min <= r <= msg.range_max):
                norm_angle = math.atan2(math.sin(angle), math.cos(angle))
                # Compute point coordinates relative to robot base_footprint (0,0)
                px = self.lidar_x_offset + r * math.cos(norm_angle)
                py = r * math.sin(norm_angle)
                d_robot = math.hypot(px, py)
                angle_from_base = math.atan2(py, px)

                if px > 0.0 and abs(angle_from_base) <= half_fov:
                    valid_distances.append(d_robot)
            angle += msg.angle_increment

        if valid_distances:
            self.min_obstacle_distance = min(valid_distances)
        else:
            self.min_obstacle_distance = float('inf')

    def safety_stop_callback(self, msg: Bool):
        if msg.data != self.manual_safety_active:
            self.manual_safety_active = msg.data
            self.has_fresh_raw_cmd = False
            if self.manual_safety_active:
                self.get_logger().warn('🛑 Safety ACTIVE (/safety_stop: true) -> Robot stopped & movement blocked!')
            else:
                self.get_logger().info('🟢 Safety INACTIVE (/safety_stop: false) -> Movement unlocked. Waiting for fresh /cmd_vel_raw.')

    def cmd_raw_callback(self, msg: Twist):
        self.raw_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        self.has_fresh_raw_cmd = True

    def control_loop(self):
        # 1. Evaluate safety zone state and log ONLY on genuine state change
        previous_state = self.zone_state
        if self.min_obstacle_distance <= self.red_dist:
            self.zone_state = "RED"
        elif self.min_obstacle_distance <= self.yellow_dist:
            self.zone_state = "YELLOW"
        else:
            self.zone_state = "GREEN"

        if self.zone_state != previous_state:
            if self.zone_state == "RED":
                self.get_logger().warn(f'🔴 RED ZONE: Obstacle at {self.min_obstacle_distance:.2f} m -> EMERGENCY STOP ACTIVATED!')
            elif self.zone_state == "YELLOW":
                self.get_logger().info(f'🟡 YELLOW ZONE: Obstacle at {self.min_obstacle_distance:.2f} m -> Velocity moderated by 2x.')
            else:
                self.get_logger().info(f'🟢 GREEN ZONE: Path Clear ({self.min_obstacle_distance:.2f} m) -> Full speed restored.')

        safe_cmd = Twist()

        if self.manual_safety_active or not self.has_fresh_raw_cmd:
            # Safety protection active OR waiting for fresh velocity after release: output zero
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0
        else:
            # 1. Command Timeout Watchdog: if no command received for > 0.5s, clear command
            cmd_age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if cmd_age <= 0.5:
                if not self.safety_zone_enabled:
                    # Bypass mode: pass-through at 100% full speed
                    safe_cmd = self.raw_cmd
                elif self.zone_state == "RED":
                    # Red Zone: FULL EMERGENCY STOP (Zero linear & zero angular motion)
                    safe_cmd.linear.x = 0.0
                    safe_cmd.angular.z = 0.0
                elif self.zone_state == "YELLOW":
                    # Yellow Zone: Obstacle is within 2x robot size -> 2x SLOWDOWN
                    safe_cmd.linear.x = self.raw_cmd.linear.x * self.slowdown
                    safe_cmd.angular.z = self.raw_cmd.angular.z
                else:
                    # Green Zone: Clear -> FULL SPEED
                    safe_cmd.linear.x = self.raw_cmd.linear.x
                    safe_cmd.angular.z = self.raw_cmd.angular.z

        self.pub_cmd_safe.publish(safe_cmd)

        # Publish clean visual zone circles to RViz
        self.publish_zone_circles()

    def publish_zone_circles(self):
        """Publishes steady, non-blinking circular boundary rings for Red & Yellow zones."""
        marker_array = MarkerArray()
        zero_stamp = Time().to_msg()

        # 1. Red Zone Marker (Radius = 0.45m, 1x Robot Size)
        red_marker = Marker()
        red_marker.header.frame_id = 'base_footprint'
        red_marker.header.stamp = zero_stamp
        red_marker.ns = 'safety_zones'
        red_marker.id = 1
        red_marker.type = Marker.CYLINDER
        red_marker.action = Marker.ADD
        red_marker.pose.position.x = 0.0
        red_marker.pose.position.y = 0.0
        red_marker.pose.position.z = 0.005
        red_marker.pose.orientation.w = 1.0
        red_marker.scale.x = self.red_dist * 2.0
        red_marker.scale.y = self.red_dist * 2.0
        red_marker.scale.z = 0.005
        red_marker.color.r = 1.0
        red_marker.color.g = 0.05
        red_marker.color.b = 0.05
        red_marker.color.a = 0.35 if self.zone_state == "RED" else 0.18
        marker_array.markers.append(red_marker)

        # 2. Yellow Zone Marker (Radius = 0.90m, 2x Robot Size)
        yellow_marker = Marker()
        yellow_marker.header.frame_id = 'base_footprint'
        yellow_marker.header.stamp = zero_stamp
        yellow_marker.ns = 'safety_zones'
        yellow_marker.id = 2
        yellow_marker.type = Marker.CYLINDER
        yellow_marker.action = Marker.ADD
        yellow_marker.pose.position.x = 0.0
        yellow_marker.pose.position.y = 0.0
        yellow_marker.pose.position.z = 0.002
        yellow_marker.pose.orientation.w = 1.0
        yellow_marker.scale.x = self.yellow_dist * 2.0
        yellow_marker.scale.y = self.yellow_dist * 2.0
        yellow_marker.scale.z = 0.003
        yellow_marker.color.r = 1.0
        yellow_marker.color.g = 0.85
        yellow_marker.color.b = 0.0
        yellow_marker.color.a = 0.30 if self.zone_state == "YELLOW" else 0.12
        marker_array.markers.append(yellow_marker)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyZoneController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
