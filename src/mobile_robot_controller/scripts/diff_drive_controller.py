#!/usr/bin/env python3
"""
================================================================================
Mobile Robot Differential Drive Controller (Python)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Designed for ROS 2 Mobile Robotics Workshop.
This node performs:
 1. Inverse Kinematics: Translates /cmd_vel (Twist) into Left/Right wheel angular speeds.
 2. Forward Kinematics (Dead-Reckoning Odometry): Computes robot pose (x, y, theta).
 3. Joint State Publishing: Publishes wheel rotations to animate all 4 wheels in RViz.
 4. TF Broadcasting: Broadcasts the dynamic transformation 'odom' -> 'base_footprint'.
 5. Hardware Command Publishing: Publishes wheel speeds for the Arduino firmware.
================================================================================
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from tf2_ros import TransformBroadcaster


class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        # ---------------------------------------------------------------------
        # 1. Declare and Read ROS 2 Parameters
        # ---------------------------------------------------------------------
        self.declare_parameter('wheel_radius', 0.05)         # meters
        self.declare_parameter('wheel_separation', 0.457)    # meters (track width calibrated)
        self.declare_parameter('publish_rate', 50.0)         # Hz
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('encoder_cpr', 330)           # Counts per rev
        self.declare_parameter('publish_joint_states', False)

        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.L = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.cpr = self.get_parameter('encoder_cpr').get_parameter_value().integer_value
        self.publish_joint_states = self.get_parameter('publish_joint_states').get_parameter_value().bool_value
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value

        # Encoder & Joint Feedback tracking
        self.has_encoder_data = False
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.has_joint_feedback = False
        self.has_prev_joint_pos = False
        self.prev_left_joint_pos = 0.0
        self.prev_right_joint_pos = 0.0
        self.delta_s_left = 0.0
        self.delta_s_right = 0.0

        # ---------------------------------------------------------------------
        # 2. Internal Kinematic State Variables
        # ---------------------------------------------------------------------
        # Target velocities from /cmd_vel
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        # Wheel angular velocities (rad/s)
        self.wheel_speed_left = 0.0
        self.wheel_speed_right = 0.0

        # Accumulated wheel angular positions (radians, for joint_states)
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Integrated Robot Pose in Odom frame
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Timestamp tracking for numerical integration (dt)
        self.last_time = self.get_clock().now()
        self.last_cmd_vel_time = self.get_clock().now()
        self.has_cmd_vel = False
        self.stop_count = 0

        # ---------------------------------------------------------------------
        # 3. Subscribers and Publishers
        # ---------------------------------------------------------------------
        from std_msgs.msg import Int32MultiArray

        # Subscribe to velocity commands (from keyboard teleop or navigation)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher to /cmd_vel for safety watchdog auto-stop in Gazebo
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Gazebo actuator velocity command publisher
        self.wheel_cmd_gazebo_pub = self.create_publisher(Float64MultiArray, '/joint_group_velocity_controller/commands', 10)

        # Subscribe to wheel encoder ticks for closed-loop odometry
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/wheel_encoder_ticks',
            self.encoder_ticks_callback,
            10
        )

        # Subscribe to /joint_states for closed-loop odometry in simulation
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publish Odometry for navigation algorithms (SLAM / Nav2)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Publish Joint States so RViz rotates the 3D wheel meshes
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Publish Wheel Speed Commands for microcontroller / Arduino firmware
        self.wheel_cmd_pub = self.create_publisher(Float32MultiArray, '/wheel_speed_commands', 10)

        # TF Broadcaster for 'odom' -> 'base_footprint'
        self.tf_broadcaster = TransformBroadcaster(self)

        # Periodic Timer Loop (using simulation-aware ROS clock)
        self.timer = self.create_timer(1.0 / self.rate, self.update_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🟢 [ACTIVE] Python Diff Drive Kinematics Controller Activated!')
        self.get_logger().info(f'   Wheel Radius:     {self.r} m')
        self.get_logger().info(f'   Wheel Separation: {self.L} m')
        self.get_logger().info(f'   Update Frequency: {self.rate} Hz')
        self.get_logger().info('=' * 60)

    def cmd_vel_callback(self, msg: Twist):
        """Callback executed whenever a new /cmd_vel Twist message arrives."""
        if abs(msg.linear.x) < 1e-4 and abs(msg.angular.z) < 1e-4:
            if not self.has_cmd_vel:
                return
            self.cmd_linear_x = 0.0
            self.cmd_angular_z = 0.0
            self.wheel_speed_left = 0.0
            self.wheel_speed_right = 0.0
            self.has_cmd_vel = False
            return

        self.last_cmd_vel_time = self.get_clock().now()
        self.has_cmd_vel = True

        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z

        # ---------------------------------------------------------------------
        # INVERSE KINEMATICS:
        # Given desired robot linear velocity V (m/s) and angular velocity W (rad/s):
        #   V_left  = V - (W * L / 2)
        #   V_right = V + (W * L / 2)
        # Rotational speeds (rad/s):
        #   W_left  = V_left / R
        #   W_right = V_right / R
        # ---------------------------------------------------------------------
        v_left = self.cmd_linear_x - (self.cmd_angular_z * self.L / 2.0)
        v_right = self.cmd_linear_x + (self.cmd_angular_z * self.L / 2.0)

        self.wheel_speed_left = v_left / self.r
        self.wheel_speed_right = v_right / self.r

    def encoder_ticks_callback(self, msg):
        """Processes real hardware encoder tick counts."""
        if len(msg.data) < 2:
            return

        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        if self.prev_left_ticks is not None and self.prev_right_ticks is not None:
            d_left = left_ticks - self.prev_left_ticks
            d_right = right_ticks - self.prev_right_ticks

            # Distance moved by each wheel side (meters)
            meters_per_tick = (2.0 * math.pi * self.r) / float(self.cpr)
            self.delta_s_left = d_left * meters_per_tick
            self.delta_s_right = d_right * meters_per_tick
            self.has_encoder_data = True

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

    def joint_state_callback(self, msg: JointState):
        """Processes Gazebo/simulation joint states for closed-loop odometry."""
        left_pos = None
        right_pos = None

        for name, pos in zip(msg.name, msg.position):
            if name in ('front_left_wheel_joint', 'rear_left_wheel_joint'):
                left_pos = pos
            elif name in ('front_right_wheel_joint', 'rear_right_wheel_joint'):
                right_pos = pos

        if left_pos is not None and right_pos is not None:
            if self.has_prev_joint_pos:
                self.delta_s_left += (left_pos - self.prev_left_joint_pos) * self.r
                self.delta_s_right += (right_pos - self.prev_right_joint_pos) * self.r
                self.has_joint_feedback = True
            self.prev_left_joint_pos = left_pos
            self.prev_right_joint_pos = right_pos
            self.has_prev_joint_pos = True

    def update_loop(self):
        """Periodic loop: updates odometry, publishes wheel commands and TF."""
        current_time = self.get_clock().now()

        # ---------------------------------------------------------------------
        # Safety Watchdog: If /cmd_vel publisher stops, auto-stop robot after 0.5s
        # ---------------------------------------------------------------------
        if self.has_cmd_vel and (current_time - self.last_cmd_vel_time).nanoseconds / 1e9 > self.cmd_vel_timeout:
            self.cmd_linear_x = 0.0
            self.cmd_angular_z = 0.0
            self.wheel_speed_left = 0.0
            self.wheel_speed_right = 0.0
            self.has_cmd_vel = False
            self.stop_count = 25  # Actively send zero twists for 0.5s at 50Hz to halt simulation

        if self.stop_count > 0:
            self.stop_count -= 1
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)

            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.wheel_cmd_gazebo_pub.publish(stop_msg)

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        if dt > 1.0:
            self.last_time = current_time
            return
        self.last_time = current_time

        # ---------------------------------------------------------------------
        # FORWARD KINEMATICS & POSE INTEGRATION:
        # If encoder data is available, compute displacement from encoders.
        # Otherwise, fall back to velocity command dead-reckoning.
        # ---------------------------------------------------------------------
        if self.has_encoder_data and (abs(self.delta_s_left) > 1e-6 or abs(self.delta_s_right) > 1e-6):
            delta_s = (self.delta_s_right + self.delta_s_left) / 2.0
            delta_theta = (self.delta_s_right - self.delta_s_left) / self.L
            # Reset delta for next tick cycle
            self.delta_s_left = 0.0
            self.delta_s_right = 0.0

            v_robot = delta_s / dt
            w_robot = delta_theta / dt
        else:
            v_left_linear = self.wheel_speed_left * self.r
            v_right_linear = self.wheel_speed_right * self.r
            v_robot = (v_right_linear + v_left_linear) / 2.0
            w_robot = (v_right_linear - v_left_linear) / self.L

            delta_s = v_robot * dt
            delta_theta = w_robot * dt

        # Midpoint / Euler numerical integration
        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Integrate wheel angular positions for RViz wheel spinning animation
        self.left_wheel_pos += self.wheel_speed_left * dt
        self.right_wheel_pos += self.wheel_speed_right * dt

        # Convert Euler yaw (theta) to Quaternion: (qx, qy, qz, qw)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # ---------------------------------------------------------------------
        # 1. Publish TF: 'odom' -> 'base_footprint'
        # ---------------------------------------------------------------------
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # ---------------------------------------------------------------------
        # 2. Publish /odom Topic
        # ---------------------------------------------------------------------
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose in odom frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Accurate small covariance to prevent huge RViz covariance bubbles
        odom_msg.pose.covariance = [
            0.001, 0.0,   0.0, 0.0, 0.0, 0.0,
            0.0,   0.001, 0.0, 0.0, 0.0, 0.0,
            0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
            0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
            0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
            0.0,   0.0,   0.0, 0.0, 0.0, 0.01
        ]

        # Velocity in base_footprint frame
        odom_msg.twist.twist.linear.x = v_robot
        odom_msg.twist.twist.angular.z = w_robot
        odom_msg.twist.covariance = odom_msg.pose.covariance

        self.odom_pub.publish(odom_msg)

        # ---------------------------------------------------------------------
        # 3. Publish /joint_states Topic (all 4 wheels)
        # ---------------------------------------------------------------------
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        joint_state.position = [
            self.left_wheel_pos,
            self.right_wheel_pos,
            self.left_wheel_pos,
            self.right_wheel_pos
        ]
        joint_state.velocity = [
            self.wheel_speed_left,
            self.wheel_speed_right,
            self.wheel_speed_left,
            self.wheel_speed_right
        ]
        if self.publish_joint_states:
            self.joint_state_pub.publish(joint_state)

        # ---------------------------------------------------------------------
        # 4. Publish /wheel_speed_commands (for Arduino firmware)
        # ---------------------------------------------------------------------
        wheel_cmds = Float32MultiArray()
        # [left_rad_s, right_rad_s]
        wheel_cmds.data = [float(self.wheel_speed_left), float(self.wheel_speed_right)]
        self.wheel_cmd_pub.publish(wheel_cmds)

        # 5. Publish to Gazebo actuator controller (/joint_group_velocity_controller/commands)
        gazebo_cmds = Float64MultiArray()
        gazebo_cmds.data = [
            float(self.wheel_speed_left),
            float(self.wheel_speed_right),
            float(self.wheel_speed_left),
            float(self.wheel_speed_right)
        ]
        self.wheel_cmd_gazebo_pub.publish(gazebo_cmds)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
