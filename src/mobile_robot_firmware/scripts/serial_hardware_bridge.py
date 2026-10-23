#!/usr/bin/env python3
"""
================================================================================
Mobile Robot Serial Hardware Bridge (Encoders + IMU Telemetry)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Bridges ROS 2 topics with the microcontroller (Arduino Uno / ESP32) over USB Serial.

Capabilities:
 1. Bidirectional Communication:
    - Transmits /wheel_speed_commands (rad/s) converted to motor PWM ("L:<pwm>,R:<pwm>\n").
    - Receives encoder ticks ("E:<left_ticks>,<right_ticks>\n") and publishes to /wheel_encoder_ticks.
    - Receives IMU readings ("I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n") and publishes to /imu/data_raw.
 2. Seamless Mock Fallback:
    - If Arduino/ESP32 is not plugged in, node enters MOCK MODE.
    - In Mock Mode, it simulates encoder counts based on commanded speeds so all
      downstream odometry nodes and RViz plugins function identically!
================================================================================
"""

import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import Imu

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class SerialHardwareBridge(Node):
    def __init__(self):
        super().__init__('serial_hardware_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_wheel_speed', 10.0)  # rad/s corresponding to 255 PWM
        self.declare_parameter('deadband_pwm', 30)       # Minimum PWM to overcome motor stiction
        self.declare_parameter('encoder_cpr', 330)       # Counts per revolution
        self.declare_parameter('wheel_radius', 0.05)     # Meters

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_wheel_speed').get_parameter_value().double_value
        self.deadband = self.get_parameter('deadband_pwm').get_parameter_value().integer_value
        self.encoder_cpr = self.get_parameter('encoder_cpr').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        self.serial_conn = None
        self.is_connected = False

        # Mock simulation state
        self.mock_left_ticks = 0.0
        self.mock_right_ticks = 0.0
        self.last_left_speed = 0.0
        self.last_right_speed = 0.0
        self.last_mock_time = self.get_clock().now()

        # Connect to Hardware (or fallback to Mock)
        self.connect_serial()

        # Publishers
        self.pub_encoder_ticks = self.create_publisher(Int32MultiArray, '/wheel_encoder_ticks', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)

        # Subscriber to wheel speeds from Kinematics Controller
        self.sub_speeds = self.create_subscription(
            Float32MultiArray,
            '/wheel_speed_commands',
            self.wheel_speed_callback,
            10
        )

        # Timer to read serial telemetry or update mock simulation (50 Hz)
        self.timer = self.create_timer(0.02, self.telemetry_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🔌 Serial Hardware Bridge Initialized')
        self.get_logger().info(f'   Target Serial Port: {self.port} @ {self.baudrate} baud')
        self.get_logger().info(f'   Hardware Status:    {"CONNECTED (Arduino/ESP32) ✅" if self.is_connected else "MOCK SIMULATION MODE 🟡"}')
        self.get_logger().info(f'   Published Topics:   /wheel_encoder_ticks, /imu/data_raw')
        self.get_logger().info('=' * 60)

    def connect_serial(self):
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('⚠️  pyserial not available. Running in MOCK mode.')
            self.is_connected = False
            return

        # Check alternative ports if /dev/ttyUSB0 doesn't exist
        port_to_try = self.port
        if not os.path.exists(port_to_try):
            for alt in ['/dev/ttyACM0', '/dev/ttyUSB1', '/dev/ttyACM1']:
                if os.path.exists(alt):
                    port_to_try = alt
                    self.port = alt
                    break

        if not os.path.exists(port_to_try):
            self.get_logger().warn(
                f'⚠️  Port {port_to_try} not found. Running in MOCK mode.\n'
                f'   (Connect Arduino/ESP32 via USB: ls /dev/ttyUSB* /dev/ttyACM*)'
            )
            self.is_connected = False
            return

        try:
            self.serial_conn = serial.Serial(
                port=port_to_try,
                baudrate=self.baudrate,
                timeout=0.05
            )
            time.sleep(1.5)  # Allow bootloader reset
            self.is_connected = True
            self.get_logger().info(f'✅ Connected to Microcontroller on {port_to_try}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to open port {port_to_try}: {e}')
            self.is_connected = False

    def rad_s_to_pwm(self, rad_s: float) -> int:
        if abs(rad_s) < 0.05:
            return 0
        ratio = rad_s / self.max_speed
        pwm = int(ratio * 255.0)
        if pwm > 0:
            pwm = max(self.deadband, min(255, pwm))
        elif pwm < 0:
            pwm = min(-self.deadband, max(-255, pwm))
        return pwm

    def wheel_speed_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return

        left_rad_s = msg.data[0]
        right_rad_s = msg.data[1]

        self.last_left_speed = left_rad_s
        self.last_right_speed = right_rad_s

        left_pwm = self.rad_s_to_pwm(left_rad_s)
        right_pwm = self.rad_s_to_pwm(right_rad_s)

        cmd_string = f"L:{left_pwm},R:{right_pwm}\n"

        if self.is_connected and self.serial_conn:
            try:
                self.serial_conn.write(cmd_string.encode('ascii'))
                self.serial_conn.flush()
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')

    def telemetry_loop(self):
        """Reads hardware serial messages or simulates mock encoder telemetry."""
        now = self.get_clock().now()

        if self.is_connected and self.serial_conn:
            # Read incoming lines from Arduino / ESP32
            try:
                while self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('E:'):
                        # Format: E:<left_ticks>,<right_ticks>
                        parts = line[2:].split(',')
                        if len(parts) == 2:
                            msg = Int32MultiArray()
                            msg.data = [int(parts[0]), int(parts[1])]
                            self.pub_encoder_ticks.publish(msg)
                    elif line.startswith('I:'):
                        # Format: I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>
                        parts = line[2:].split(',')
                        if len(parts) == 6:
                            imu_msg = Imu()
                            imu_msg.header.stamp = now.to_msg()
                            imu_msg.header.frame_id = 'imu_link'
                            # Check if values are already float (SI) or integer ADC counts
                            try:
                                ax_val = float(parts[0])
                                ay_val = float(parts[1])
                                az_val = float(parts[2])
                                gx_val = float(parts[3])
                                gy_val = float(parts[4])
                                gz_val = float(parts[5])

                                if abs(ax_val) > 50 or abs(ay_val) > 50 or abs(az_val) > 50:
                                    # Raw 16-bit counts
                                    imu_msg.linear_acceleration.x = (ax_val / 16384.0) * 9.80665
                                    imu_msg.linear_acceleration.y = (ay_val / 16384.0) * 9.80665
                                    imu_msg.linear_acceleration.z = (az_val / 16384.0) * 9.80665
                                    deg_to_rad = 3.14159265 / 180.0
                                    imu_msg.angular_velocity.x = (gx_val / 131.0) * deg_to_rad
                                    imu_msg.angular_velocity.y = (gy_val / 131.0) * deg_to_rad
                                    imu_msg.angular_velocity.z = (gz_val / 131.0) * deg_to_rad
                                else:
                                    # Already scaled SI values
                                    imu_msg.linear_acceleration.x = ax_val
                                    imu_msg.linear_acceleration.y = ay_val
                                    imu_msg.linear_acceleration.z = az_val
                                    imu_msg.angular_velocity.x = gx_val
                                    imu_msg.angular_velocity.y = gy_val
                                    imu_msg.angular_velocity.z = gz_val

                                self.pub_imu.publish(imu_msg)
                            except ValueError:
                                pass
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
        else:
            # Mock mode: Integrate wheel angular velocities into ticks
            dt = (now - self.last_mock_time).nanoseconds / 1e9
            self.last_mock_time = now

            if dt > 0:
                # ticks = (rad_s * dt / (2 * pi)) * CPR
                two_pi = 6.28318530718
                self.mock_left_ticks += (self.last_left_speed * dt / two_pi) * self.encoder_cpr
                self.mock_right_ticks += (self.last_right_speed * dt / two_pi) * self.encoder_cpr

                msg = Int32MultiArray()
                msg.data = [int(self.mock_left_ticks), int(self.mock_right_ticks)]
                self.pub_encoder_ticks.publish(msg)

                # Mock IMU telemetry (gravity on Z, zero angular rate)
                imu_msg = Imu()
                imu_msg.header.stamp = now.to_msg()
                imu_msg.header.frame_id = 'imu_link'
                imu_msg.linear_acceleration.z = 9.80665
                self.pub_imu.publish(imu_msg)

    def destroy_node(self):
        if self.is_connected and self.serial_conn:
            try:
                self.serial_conn.write(b"L:0,R:0\n")
                self.serial_conn.flush()
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialHardwareBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
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
