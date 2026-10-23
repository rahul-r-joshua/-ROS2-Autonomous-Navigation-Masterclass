/**
 * ============================================================================
 * 4-Wheel Differential Drive Robot - Arduino Firmware (Encoders + IMU)
 * ============================================================================
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 * 
 * Workshop Edition for ROS 2 Mobile Robotics Workshop.
 * 
 * Features:
 *  1. Dual H-Bridge Motor Control (L298N / TB6612FNG).
 *  2. DC Motor Quadrature Encoders via Hardware Interrupts (Pins 2 & 3).
 *  3. MPU6050 6-DOF IMU integration via I2C (Pins A4-SDA, A5-SCL).
 *  4. 1.0s Watchdog timer for fail-safe emergency motor stopping.
 *  5. High-speed bidirectional Serial communication with ROS 2 @ 115200 baud.
 * 
 * Serial Protocol:
 *  - Incoming (from ROS 2): "L:<left_pwm>,R:<right_pwm>\n"   (-255 to 255)
 *  - Outgoing (to ROS 2):   "E:<left_ticks>,<right_ticks>\n" (Encoder Feedback)
 *                           "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n" (Raw IMU Feedback)
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
// 2. Encoder Pin Definitions & Variables
// -----------------------------------------------------------------------------
const int ENC_LEFT_A  = 2;  // External Interrupt INT0
const int ENC_LEFT_B  = 4;  // Direction logic
const int ENC_RIGHT_A = 3;  // External Interrupt INT1
const int ENC_RIGHT_B = 11; // Direction logic

volatile long left_encoder_ticks  = 0;
volatile long right_encoder_ticks = 0;

// -----------------------------------------------------------------------------
// 3. IMU Configuration (Supports both MPU6050 and BNO055)
// -----------------------------------------------------------------------------
const int MPU6050_ADDR = 0x68;
const int BNO055_ADDR  = 0x28;

enum ImuType { IMU_NONE, IMU_MPU6050, IMU_BNO055 };
ImuType active_imu = IMU_NONE;

// -----------------------------------------------------------------------------
// 4. Timing & Watchdog
// -----------------------------------------------------------------------------
const unsigned long TIMEOUT_MS = 1000;    // 1 second safety watchdog
const unsigned long TELEMETRY_MS = 50;   // Send encoder & IMU feedback at 20 Hz
unsigned long last_cmd_time = 0;
unsigned long last_telemetry_time = 0;

// -----------------------------------------------------------------------------
// Interrupt Service Routines (ISRs) for Encoders
// -----------------------------------------------------------------------------
void isrLeftEncoder() {
  if (digitalRead(ENC_LEFT_B) == HIGH) {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
}

void isrRightEncoder() {
  if (digitalRead(ENC_RIGHT_B) == HIGH) {
    right_encoder_ticks++;
  } else {
    right_encoder_ticks--;
  }
}

// -----------------------------------------------------------------------------
// Helper Motor Declarations
// -----------------------------------------------------------------------------
void setLeftMotors(int pwm_val);
void setRightMotors(int pwm_val);
void stopMotors();
void parseAndExecuteCommand(String cmd);
void sendTelemetry();

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // Configure Motor Pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Stop motors initially
  stopMotors();

  // Configure Encoder Pins with Internal Pullups
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightEncoder, RISING);

  // Initialize I2C and detect IMU (MPU6050 or BNO055)
  Wire.begin();
  
  // 1. Try MPU6050 (0x68)
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    active_imu = IMU_MPU6050;
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    Wire.endTransmission();
  } else {
    // 2. Try Bosch BNO055 (0x28)
    Wire.beginTransmission(BNO055_ADDR);
    if (Wire.endTransmission() == 0) {
      active_imu = IMU_BNO055;
      Wire.beginTransmission(BNO055_ADDR);
      Wire.write(0x3D); // OPR_MODE register
      Wire.write(0x08); // IMU mode
      Wire.endTransmission();
    }
  }

  Serial.println("==================================================");
  Serial.println("✓ 4-Wheel Robot Arduino Firmware Ready!");
  Serial.print("  Encoders: Active on INT0 (Pin 2) & INT1 (Pin 3)\n");
  Serial.print("  IMU: ");
  if (active_imu == IMU_MPU6050) {
    Serial.println("MPU-6050 Detected & Initialized ✓");
  } else if (active_imu == IMU_BNO055) {
    Serial.println("Bosch BNO-055 Detected & Initialized ✓");
  } else {
    Serial.println("None Detected (Skipped)");
  }
  Serial.println("  Safety Watchdog: 1000 ms");
  Serial.println("==================================================");

  last_cmd_time = millis();
  last_telemetry_time = millis();
}

// -----------------------------------------------------------------------------
// Main Loop
// -----------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // 1. Process incoming commands from ROS 2
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      parseAndExecuteCommand(input);
      last_cmd_time = now;
    }
  }

  // 2. Safety Watchdog: Stop motors if ROS 2 freezes or disconnects
  if (now - last_cmd_time > TIMEOUT_MS) {
    stopMotors();
  }

  // 3. Periodic Telemetry (Encoder Ticks & IMU data at 20 Hz)
  if (now - last_telemetry_time >= TELEMETRY_MS) {
    last_telemetry_time = now;
    sendTelemetry();
  }
}

// -----------------------------------------------------------------------------
// Parse ROS 2 Serial Command: "L:<left>,R:<right>"
// -----------------------------------------------------------------------------
void parseAndExecuteCommand(String cmd) {
  int l_index = cmd.indexOf("L:");
  int r_index = cmd.indexOf("R:");

  if (l_index != -1 && r_index != -1) {
    int comma_index = cmd.indexOf(',');
    if (comma_index != -1) {
      String l_str = cmd.substring(l_index + 2, comma_index);
      String r_str = cmd.substring(r_index + 2);

      int left_pwm  = l_str.toInt();
      int right_pwm = r_str.toInt();

      setLeftMotors(left_pwm);
      setRightMotors(right_pwm);
    }
  }
}

// -----------------------------------------------------------------------------
// Send Telemetry (Encoders + IMU) to ROS 2
// -----------------------------------------------------------------------------
void sendTelemetry() {
  // Read atomic snapshot of encoder ticks
  noInterrupts();
  long l_ticks = left_encoder_ticks;
  long r_ticks = right_encoder_ticks;
  interrupts();

  // Output format: "E:<left_ticks>,<right_ticks>"
  Serial.print("E:");
  Serial.print(l_ticks);
  Serial.print(",");
  Serial.println(r_ticks);

  // Send IMU data if detected (MPU6050 or BNO055)
  if (active_imu == IMU_MPU6050) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // Register 0x3B (ACCEL_XOUT_H)
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU6050_ADDR, 14, true) == 14) {
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      int16_t temp = (Wire.read() << 8) | Wire.read(); (void)temp;
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();

      // Output format: "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>"
      Serial.print("I:");
      Serial.print(ax); Serial.print(",");
      Serial.print(ay); Serial.print(",");
      Serial.print(az); Serial.print(",");
      Serial.print(gx); Serial.print(",");
      Serial.print(gy); Serial.print(",");
      Serial.println(gz);
    }
  } else if (active_imu == IMU_BNO055) {
    // Read Linear Accel (0x08) and Gyro (0x14) from BNO055
    Wire.beginTransmission(BNO055_ADDR);
    Wire.write(0x08); // ACC_DATA_X_LSB
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(BNO055_ADDR, 6, true) == 6) {
      int16_t ax = Wire.read() | (Wire.read() << 8);
      int16_t ay = Wire.read() | (Wire.read() << 8);
      int16_t az = Wire.read() | (Wire.read() << 8);

      Wire.beginTransmission(BNO055_ADDR);
      Wire.write(0x14); // GYR_DATA_X_LSB
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom(BNO055_ADDR, 6, true) == 6) {
        int16_t gx = Wire.read() | (Wire.read() << 8);
        int16_t gy = Wire.read() | (Wire.read() << 8);
        int16_t gz = Wire.read() | (Wire.read() << 8);

        Serial.print("I:");
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.println(gz);
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Motor Control Functions
// -----------------------------------------------------------------------------
void setLeftMotors(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(pwm_val));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setRightMotors(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(pwm_val));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
