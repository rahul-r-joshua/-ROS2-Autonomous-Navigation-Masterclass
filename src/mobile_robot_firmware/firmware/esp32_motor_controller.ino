/**
 * ============================================================================
 * 4-Wheel Differential Drive Robot - ESP32 Firmware (Encoders + IMU)
 * ============================================================================
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 * 
 * Workshop Edition for ROS 2 Mobile Robotics Workshop.
 * 
 * Hardware Compatibility:
 *  - ESP32 Development Board (30-pin or 38-pin ESP32-WROOM-32)
 *  - Motor Driver: L298N / TB6612FNG Dual H-Bridge
 *  - DC Gearmotors with Quadrature Encoders
 *  - MPU6050 6-Axis IMU (I2C)
 * 
 * Pinout Assignments (ESP32):
 *  - Left Motor:   ENA=GPIO 25 (PWM), IN1=GPIO 26, IN2=GPIO 27
 *  - Right Motor:  ENB=GPIO 14 (PWM), IN3=GPIO 12, IN4=GPIO 13
 *  - Left Encoder: ENCA=GPIO 18, ENCB=GPIO 19
 *  - Right Encoder:ENCA=GPIO 16, ENCB=GPIO 17
 *  - I2C IMU:      SDA=GPIO 21,  SCL=GPIO 22
 * ============================================================================
 */

#include <Wire.h>

// -----------------------------------------------------------------------------
// 1. Motor Pin Definitions
// -----------------------------------------------------------------------------
const int PIN_ENA = 25;   // Left Motor PWM
const int PIN_IN1 = 26;   // Left Direction 1
const int PIN_IN2 = 27;   // Left Direction 2

const int PIN_ENB = 14;   // Right Motor PWM
const int PIN_IN3 = 12;   // Right Direction 1
const int PIN_IN4 = 13;   // Right Direction 2

// ESP32 PWM (LEDC) Configuration
const int PWM_FREQ = 1000;       // 1 kHz PWM frequency
const int PWM_RES  = 8;          // 8-bit resolution (0-255)
const int LEDC_CH_LEFT  = 0;     // PWM Channel 0
const int LEDC_CH_RIGHT = 1;     // PWM Channel 1

// -----------------------------------------------------------------------------
// 2. Encoder Pin Definitions & Counters
// -----------------------------------------------------------------------------
const int PIN_ENC_LEFT_A  = 18;  // Interrupt pin
const int PIN_ENC_LEFT_B  = 19;
const int PIN_ENC_RIGHT_A = 16;  // Interrupt pin
const int PIN_ENC_RIGHT_B = 17;

volatile long left_encoder_ticks  = 0;
volatile long right_encoder_ticks = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// -----------------------------------------------------------------------------
// 3. IMU Configuration (Supports both MPU6050 and BNO055)
// -----------------------------------------------------------------------------
const int MPU6050_ADDR = 0x68;
const int BNO055_ADDR  = 0x28;

enum ImuType { IMU_NONE, IMU_MPU6050, IMU_BNO055 };
ImuType active_imu = IMU_NONE;

// -----------------------------------------------------------------------------
// 4. Timing & Safety Watchdog
// -----------------------------------------------------------------------------
const unsigned long TIMEOUT_MS    = 1000;  // Stop motors if no command for 1 second
const unsigned long TELEMETRY_MS  = 50;    // Telemetry rate 20 Hz
unsigned long last_cmd_time       = 0;
unsigned long last_telemetry_time = 0;

// -----------------------------------------------------------------------------
// Helper Declarations
// -----------------------------------------------------------------------------
void setLeftMotors(int pwm_val);
void setRightMotors(int pwm_val);
void stopMotors();
void parseAndExecuteCommand(String cmd);
void sendTelemetry();

// -----------------------------------------------------------------------------
// Interrupt Service Routines (ISRs) for ESP32
// -----------------------------------------------------------------------------
void IRAM_ATTR isrLeftEncoder() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(PIN_ENC_LEFT_B) == HIGH) {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isrRightEncoder() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(PIN_ENC_RIGHT_B) == HIGH) {
    right_encoder_ticks++;
  } else {
    right_encoder_ticks--;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  // Configure Motor Direction Pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Configure ESP32 LEDC PWM Channels
  ledcSetup(LEDC_CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_ENA, LEDC_CH_LEFT);

  ledcSetup(LEDC_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_ENB, LEDC_CH_RIGHT);

  stopMotors();

  // Configure Encoder Pins with Pullups
  pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_LEFT_B, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), isrLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), isrRightEncoder, RISING);

  // Initialize I2C on standard ESP32 pins (SDA=21, SCL=22)
  Wire.begin(21, 22);
  
  // 1. Try MPU6050 (0x68)
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    active_imu = IMU_MPU6050;
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
  } else {
    // 2. Try Bosch BNO055 (0x28)
    Wire.beginTransmission(BNO055_ADDR);
    if (Wire.endTransmission() == 0) {
      active_imu = IMU_BNO055;
      Wire.beginTransmission(BNO055_ADDR);
      Wire.write(0x3D); // OPR_MODE
      Wire.write(0x08); // IMU mode
      Wire.endTransmission();
    }
  }

  Serial.println("==================================================");
  Serial.println("✓ 4-Wheel Robot ESP32 Firmware Ready!");
  Serial.println("  LEDC PWM Channels 0 & 1 Initialized (1 kHz)");
  Serial.println("  Hardware Interrupt Encoders (GPIO 18, 19, 16, 17)");
  Serial.print("  IMU: ");
  if (active_imu == IMU_MPU6050) {
    Serial.println("MPU-6050 Detected & Initialized ✓");
  } else if (active_imu == IMU_BNO055) {
    Serial.println("Bosch BNO-055 Detected & Initialized ✓");
  } else {
    Serial.println("None Detected (Skipped)");
  }
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

  // 2. Safety Watchdog
  if (now - last_cmd_time > TIMEOUT_MS) {
    stopMotors();
  }

  // 3. Send Telemetry to ROS 2 (20 Hz)
  if (now - last_telemetry_time >= TELEMETRY_MS) {
    last_telemetry_time = now;
    sendTelemetry();
  }
}

// -----------------------------------------------------------------------------
// Parse Serial Command: "L:<left>,R:<right>"
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
// Send Telemetry (Encoders + IMU)
// -----------------------------------------------------------------------------
void sendTelemetry() {
  portENTER_CRITICAL(&mux);
  long l_ticks = left_encoder_ticks;
  long r_ticks = right_encoder_ticks;
  portEXIT_CRITICAL(&mux);

  // Send encoder ticks: "E:<left_ticks>,<right_ticks>"
  Serial.print("E:");
  Serial.print(l_ticks);
  Serial.print(",");
  Serial.println(r_ticks);

  // Send IMU data if detected (MPU6050 or BNO055)
  if (active_imu == IMU_MPU6050) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU6050_ADDR, 14, true) == 14) {
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      int16_t temp = (Wire.read() << 8) | Wire.read(); (void)temp;
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();

      Serial.print("I:");
      Serial.print(ax); Serial.print(",");
      Serial.print(ay); Serial.print(",");
      Serial.print(az); Serial.print(",");
      Serial.print(gx); Serial.print(",");
      Serial.print(gy); Serial.print(",");
      Serial.println(gz);
    }
  } else if (active_imu == IMU_BNO055) {
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
// Motor Control (ESP32 LEDC PWM)
// -----------------------------------------------------------------------------
void setLeftMotors(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(LEDC_CH_LEFT, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    ledcWrite(LEDC_CH_LEFT, abs(pwm_val));
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(LEDC_CH_LEFT, 0);
  }
}

void setRightMotors(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    ledcWrite(LEDC_CH_RIGHT, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    ledcWrite(LEDC_CH_RIGHT, abs(pwm_val));
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    ledcWrite(LEDC_CH_RIGHT, 0);
  }
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  ledcWrite(LEDC_CH_LEFT, 0);
  ledcWrite(LEDC_CH_RIGHT, 0);
}
