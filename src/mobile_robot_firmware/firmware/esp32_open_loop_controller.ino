/**
 * ============================================================================
 * 4-Wheel Differential Drive Robot - ESP32 Firmware (WITHOUT ENCODERS / OPEN-LOOP + IMU)
 * ============================================================================
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 * 
 * Workshop Edition for ROS 2 Mobile Robotics Workshop.
 * 
 * Use this sketch when deploying on ESP32 without quadrature motor encoders
 * (e.g., standard TT DC gearmotors or open-loop skid-steer chassis).
 * 
 * Features:
 *  1. Dual H-Bridge Motor Control (L298N / TB6612FNG).
 *  2. High-precision hardware LEDC PWM (compatible with ESP32 Core 2.x & 3.x).
 *  3. Open-loop velocity control directly mapped from ROS 2 Twist / PWM commands.
 *  4. MPU6050 / BNO055 IMU integration on I2C (Pins GPIO 21-SDA, GPIO 22-SCL).
 *  5. 1.0s Watchdog timer for fail-safe automatic motor cutoff on serial loss.
 *  6. 115200 Baud high-speed bidirectional communication with serial_hardware_bridge.py.
 * 
 * Serial Protocol:
 *  - Incoming (from ROS 2): "L:<left_pwm>,R:<right_pwm>\n"   (-255 to 255)
 *  - Outgoing (to ROS 2):   "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n" (Raw IMU Feedback)
 * ============================================================================
 */

#include <Wire.h>

// -----------------------------------------------------------------------------
// 1. ESP32 Pin Definitions
// -----------------------------------------------------------------------------
// Left Motor Channel
const int PIN_ENA = 25;   // Left PWM Speed Pin
const int PIN_IN1 = 26;   // Left Direction 1
const int PIN_IN2 = 27;   // Left Direction 2

// Right Motor Channel
const int PIN_ENB = 14;   // Right PWM Speed Pin
const int PIN_IN3 = 12;   // Right Direction 1
const int PIN_IN4 = 13;   // Right Direction 2

// I2C IMU Pins
const int PIN_SDA = 21;
const int PIN_SCL = 22;

// LEDC Hardware PWM Configuration
const int PWM_FREQ = 20000;    // 20 kHz ultrasonic (silent motors)
const int PWM_RESOLUTION = 8;  // 8-bit resolution (0 - 255)
const int PWM_CH_LEFT = 0;
const int PWM_CH_RIGHT = 1;

// -----------------------------------------------------------------------------
// 2. IMU Configuration
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
// 4. Helper: LEDC PWM write abstraction (Supports ESP32 Core 2.x & 3.x)
// -----------------------------------------------------------------------------
void initPwmPin(int pin, int channel) {
#if defined(ESP_ARDUINO_VERSION_VAL) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(pin, PWM_FREQ, PWM_RESOLUTION);
#else
  ledcSetup(channel, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pin, channel);
#endif
}

void writePwm(int pin, int channel, int duty) {
  duty = constrain(duty, 0, 255);
#if defined(ESP_ARDUINO_VERSION_VAL) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// -----------------------------------------------------------------------------
// 5. Motor Control Functions
// -----------------------------------------------------------------------------
void setMotorSpeeds(int left_pwm, int right_pwm) {
  // Left Motor Direction
  if (left_pwm > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else if (left_pwm < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }
  writePwm(PIN_ENA, PWM_CH_LEFT, abs(left_pwm));

  // Right Motor Direction
  if (right_pwm > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else if (right_pwm < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
  }
  writePwm(PIN_ENB, PWM_CH_RIGHT, abs(right_pwm));
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  writePwm(PIN_ENA, PWM_CH_LEFT, 0);
  writePwm(PIN_ENB, PWM_CH_RIGHT, 0);
}

// -----------------------------------------------------------------------------
// 6. Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  // Direction Pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // PWM Pins
  initPwmPin(PIN_ENA, PWM_CH_LEFT);
  initPwmPin(PIN_ENB, PWM_CH_RIGHT);

  stopMotors();

  // I2C for IMU
  Wire.begin(PIN_SDA, PIN_SCL);
  delay(100);

  // Detect MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    active_imu = IMU_MPU6050;
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    Wire.endTransmission();
    Serial.println("STATUS:ESP32_OPEN_LOOP_MPU6050_READY");
  } else {
    // Detect BNO055
    Wire.beginTransmission(BNO055_ADDR);
    if (Wire.endTransmission() == 0) {
      active_imu = IMU_BNO055;
      Serial.println("STATUS:ESP32_OPEN_LOOP_BNO055_READY");
    } else {
      active_imu = IMU_NONE;
      Serial.println("STATUS:ESP32_OPEN_LOOP_NO_IMU");
    }
  }

  last_command_time = millis();
}

// -----------------------------------------------------------------------------
// 7. Loop: Command Processing, Watchdog & Telemetry
// -----------------------------------------------------------------------------
void loop() {
  // 1. Process Serial Commands from ROS 2
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("L:") && line.indexOf(",R:") > 0) {
      int r_idx = line.indexOf(",R:");
      int left_pwm = line.substring(2, r_idx).toInt();
      int right_pwm = line.substring(r_idx + 3).toInt();

      left_pwm = constrain(left_pwm, -255, 255);
      right_pwm = constrain(right_pwm, -255, 255);

      setMotorSpeeds(left_pwm, right_pwm);
      last_command_time = millis();
    }
  }

  // 2. Failsafe Watchdog Check
  if (millis() - last_command_time > WATCHDOG_TIMEOUT_MS) {
    stopMotors();
  }

  // 3. Publish Telemetry at 50 Hz
  if (millis() - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_time = millis();

    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;

    if (active_imu == IMU_MPU6050) {
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDR, 14, true);

      if (Wire.available() >= 14) {
        ax = (Wire.read() << 8) | Wire.read();
        ay = (Wire.read() << 8) | Wire.read();
        az = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Skip temp
        gx = (Wire.read() << 8) | Wire.read();
        gy = (Wire.read() << 8) | Wire.read();
        gz = (Wire.read() << 8) | Wire.read();
      }
    }

    Serial.print("I:");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
  }
}
