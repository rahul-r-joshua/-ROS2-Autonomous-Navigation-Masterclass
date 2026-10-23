/**
 * ============================================================================
 * 4-Wheel Differential Drive Robot - Arduino Firmware (WITHOUT ENCODERS / OPEN-LOOP + IMU)
 * ============================================================================
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 * 
 * Workshop Edition for ROS 2 Mobile Robotics Workshop.
 * 
 * Use this sketch when your 4WD mobile robot does NOT have motor encoders
 * (e.g. standard yellow TT DC gearmotors without Hall sensors).
 * 
 * Features:
 *  1. Dual H-Bridge Motor Control (L298N).
 *  2. Open-Loop PWM control driven directly by /wheel_speed_commands from ROS 2.
 *  3. MPU6050 / BNO055 IMU integration via I2C (Pins A4-SDA, A5-SCL) with auto-detection.
 *  4. 1.0s Watchdog timer for fail-safe emergency motor stopping if ROS 2 disconnects.
 *  5. High-speed bidirectional Serial communication with ROS 2 @ 115200 baud.
 * 
 * Serial Protocol:
 *  - Incoming (from ROS 2): "L:<left_pwm>,R:<right_pwm>\n"   (-255 to 255)
 *  - Outgoing (to ROS 2):   "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n" (Raw IMU Feedback)
 * ============================================================================
 */

#include <Wire.h>

// -----------------------------------------------------------------------------
// 1. Motor Pin Definitions (L298N)
// -----------------------------------------------------------------------------
const int ENA = 5;    // Left Motors PWM Speed
const int IN1 = 7;    // Left Direction 1
const int IN2 = 8;    // Left Direction 2

const int ENB = 6;    // Right Motors PWM Speed
const int IN3 = 9;    // Right Direction 1
const int IN4 = 10;   // Right Direction 2

// -----------------------------------------------------------------------------
// 2. IMU Configuration (Supports both MPU6050 and BNO055)
// -----------------------------------------------------------------------------
const int MPU6050_ADDR = 0x68;
const int BNO055_ADDR  = 0x28;

enum ImuType { IMU_NONE, IMU_MPU6050, IMU_BNO055 };
ImuType active_imu = IMU_NONE;

// -----------------------------------------------------------------------------
// 3. Timing & Watchdog
// -----------------------------------------------------------------------------
unsigned long last_command_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 1000; // 1.0s fail-safe stop
unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 20; // 50 Hz telemetry

// -----------------------------------------------------------------------------
// Helper Declarations
// -----------------------------------------------------------------------------
void setLeftMotor(int pwm_val);
void setRightMotor(int pwm_val);
void stopMotors();
void processSerialCommands();
void sendImuTelemetry();

// -----------------------------------------------------------------------------
// 4. Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  // Motor Pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Stop Motors initially
  stopMotors();

  // Initialize I2C for IMU
  Wire.begin();
  delay(100);

  // Auto-detect MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    active_imu = IMU_MPU6050;
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    Wire.endTransmission();
    Serial.println("STATUS:IMU MPU6050 DETECTED (0x68)");
  } else {
    // Check BNO055
    Wire.beginTransmission(BNO055_ADDR);
    if (Wire.endTransmission() == 0) {
      active_imu = IMU_BNO055;
      Serial.println("STATUS:IMU BNO055 DETECTED (0x28)");
    } else {
      active_imu = IMU_NONE;
      Serial.println("STATUS:NO IMU DETECTED");
    }
  }

  Serial.println("STATUS:ARDUINO OPEN-LOOP MOTOR CONTROLLER READY");
}

// -----------------------------------------------------------------------------
// 5. Main Loop
// -----------------------------------------------------------------------------
void loop() {
  // 1. Process incoming serial commands from ROS 2
  processSerialCommands();

  // 2. Watchdog: Emergency stop if no command received within 1 second
  if (millis() - last_command_time > WATCHDOG_TIMEOUT_MS) {
    stopMotors();
  }

  // 3. Periodic IMU Telemetry to ROS 2 (50 Hz)
  if (millis() - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_time = millis();
    sendImuTelemetry();
  }
}

// -----------------------------------------------------------------------------
// 6. Motor Control Functions
// -----------------------------------------------------------------------------
void setLeftMotor(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwm_val);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setRightMotor(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -pwm_val);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// -----------------------------------------------------------------------------
// 7. Serial Communication with ROS 2
// -----------------------------------------------------------------------------
void processSerialCommands() {
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("L:") && cmd.indexOf(",R:") != -1) {
      int r_idx = cmd.indexOf(",R:");
      String left_str = cmd.substring(2, r_idx);
      String right_str = cmd.substring(r_idx + 3);

      int left_pwm = left_str.toInt();
      int right_pwm = right_str.toInt();

      setLeftMotor(left_pwm);
      setRightMotor(right_pwm);
      last_command_time = millis();
    }
  }
}

void sendImuTelemetry() {
  if (active_imu == IMU_MPU6050) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);

    if (Wire.available() >= 14) {
      int16_t ax = Wire.read() << 8 | Wire.read();
      int16_t ay = Wire.read() << 8 | Wire.read();
      int16_t az = Wire.read() << 8 | Wire.read();
      int16_t temp = Wire.read() << 8 | Wire.read(); (void)temp;
      int16_t gx = Wire.read() << 8 | Wire.read();
      int16_t gy = Wire.read() << 8 | Wire.read();
      int16_t gz = Wire.read() << 8 | Wire.read();

      // Convert to SI units: m/s^2 and rad/s
      float ax_m_s2 = (float)ax / 16384.0 * 9.80665;
      float ay_m_s2 = (float)ay / 16384.0 * 9.80665;
      float az_m_s2 = (float)az / 16384.0 * 9.80665;
      float gx_rad_s = (float)gx / 131.0 * (3.14159265 / 180.0);
      float gy_rad_s = (float)gy / 131.0 * (3.14159265 / 180.0);
      float gz_rad_s = (float)gz / 131.0 * (3.14159265 / 180.0);

      Serial.print("I:");
      Serial.print(ax_m_s2, 3); Serial.print(",");
      Serial.print(ay_m_s2, 3); Serial.print(",");
      Serial.print(az_m_s2, 3); Serial.print(",");
      Serial.print(gx_rad_s, 3); Serial.print(",");
      Serial.print(gy_rad_s, 3); Serial.print(",");
      Serial.println(gz_rad_s, 3);
    }
  }
}
