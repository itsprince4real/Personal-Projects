#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <ESP32Servo.h>

// --- HARDWARE PIN ASSIGNMENTS ---
#define PIN_SERVO1 18 // Front Roll
#define PIN_SERVO2 19 // Front Pitch
#define PIN_SERVO3 27 // Rear Pitch
#define PIN_SERVO4 32 // Rear Roll
#define PIN_ESC1   25 // Front Motor
#define PIN_ESC2   26 // Rear Motor

// --- SERVO NEUTRALS & SAFETY LIMITS ---
const int S1_NEUTRAL = 80, S1_MIN = 65, S1_MAX = 95;
const int S2_NEUTRAL = 95, S2_MIN = 80, S2_MAX = 110;
const int S3_NEUTRAL = 78, S3_MIN = 63, S3_MAX = 93;
const int S4_NEUTRAL = 82, S4_MIN = 67, S4_MAX = 97;

// --- SERVO DIRECTION MULTIPLIERS (-1.0 or 1.0) ---
const float DIR_S1_ROLL  = 1.0; 
const float DIR_S2_PITCH = 1.0;
const float DIR_S3_PITCH = 1.0; 
const float DIR_S4_ROLL  = 1.0;

const uint8_t MPU_ADDR = 0x68;

// --- DATA STRUCT MATCHING TRANSMITTER ---
typedef struct struct_message {
  uint16_t throttle; // 1000 - 2000 us input range from TX
  int8_t pitch;      // -30 to +30 deg target
  int8_t roll;       // -30 to +30 deg target
  int8_t yaw;        // -30 to +30 deg target
  bool armed;        // Safety arm status
} struct_message;

struct_message rxData;

Servo servo1, servo2, servo3, servo4, esc1, esc2;

// PID Gains (P-Only for baseline setup)
float Kp_pitch = 1.2;
float Kp_roll  = 1.2;
float Kp_yaw   = 0.8;

// Gyro Offsets & Angle Tracking
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float pitchAngle = 0, rollAngle = 0;
unsigned long prevTime = 0;

// ESP-NOW Receive Callback
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(rxData)) {
    memcpy(&rxData, incomingData, sizeof(rxData));
  }
}

void setup() {
  Serial.begin(115200);

  // 1. ESC HARDWARE INITIALIZATION (Force zero throttle before attaching)
  esc1.attach(PIN_ESC1, 1000, 2000);
  esc2.attach(PIN_ESC2, 1000, 2000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);

  // 2. SERVO PWM TIMERS & ATTACHMENT
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1.setPeriodHertz(50); servo1.attach(PIN_SERVO1, 500, 2500);
  servo2.setPeriodHertz(50); servo2.attach(PIN_SERVO2, 500, 2500);
  servo3.setPeriodHertz(50); servo3.attach(PIN_SERVO3, 500, 2500);
  servo4.setPeriodHertz(50); servo4.attach(PIN_SERVO4, 500, 2500);

  // Set initial neutral servo positions
  servo1.write(S1_NEUTRAL);
  servo2.write(S2_NEUTRAL);
  servo3.write(S3_NEUTRAL);
  servo4.write(S4_NEUTRAL);

  // 3. ESP-NOW SETUP
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
  }

  // 4. MPU6500 I2C INITIALIZATION
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true); // Wake up
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(true); // DLPF 42Hz
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(true); // Gyro +/-500dps
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission(true); // Accel +/-4g

  // Gyro Calibration
  delay(500);
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x43); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);
    if (Wire.available() >= 6) {
      sumX += (int16_t)((Wire.read() << 8) | Wire.read()) / 65.5;
      sumY += (int16_t)((Wire.read() << 8) | Wire.read()) / 65.5;
      sumZ += (int16_t)((Wire.read() << 8) | Wire.read()) / 65.5;
    }
    delay(5);
  }
  gyroX_offset = sumX / 200.0;
  gyroY_offset = sumY / 200.0;
  gyroZ_offset = sumZ / 200.0;

  prevTime = micros();
}

void loop() {
  unsigned long currentTime = micros();
  if (currentTime - prevTime < 5000) return; // Fixed 200Hz loop rate (5ms)
  float dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;

  // Read MPU6500 Data
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() >= 14) {
    int16_t rawAccX = (Wire.read() << 8) | Wire.read();
    int16_t rawAccY = (Wire.read() << 8) | Wire.read();
    int16_t rawAccZ = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip Temperature bytes
    int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
    int16_t rawGyroY = (Wire.read() << 8) | Wire.read();
    int16_t rawGyroZ = (Wire.read() << 8) | Wire.read();

    float accX = (float)rawAccX / 8192.0;
    float accY = (float)rawAccY / 8192.0;
    float accZ = (float)rawAccZ / 8192.0;

    float gyroX = ((float)rawGyroX / 65.5) - gyroX_offset;
    float gyroY = ((float)rawGyroY / 65.5) - gyroY_offset;
    float gyroZ = ((float)rawGyroZ / 65.5) - gyroZ_offset;

    // Complementary Filter Angle Calculation
    float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2958;
    float accRoll  = atan2(accY, accZ) * 57.2958;

    pitchAngle = 0.96 * (pitchAngle + gyroY * dt) + 0.04 * accPitch;
    rollAngle  = 0.96 * (rollAngle + gyroX * dt) + 0.04 * accRoll;

    // Calculate Error Terms and PID Corrections
    float pidPitch = Kp_pitch * (rxData.pitch - pitchAngle);
    float pidRoll  = Kp_roll  * (rxData.roll  - rollAngle);
    float pidYaw   = Kp_yaw   * (rxData.yaw   - gyroZ);

    if (rxData.armed && rxData.throttle > 1050) {
      
      // 1. BASE THROTTLE MAP: Full stick (2000us) -> 1450us base pulse
      uint16_t baseThrottle = map(rxData.throttle, 1000, 2000, 1000, 1450);

      // 2. PITCH DIFFERENTIAL THRUST MIXING
      int esc1Calculated = baseThrottle + pidPitch;
      int esc2Calculated = baseThrottle - pidPitch;

      // 3. HARD POWER CEILING: Constrain strictly to 1500us MAX (50% Motor Power)
      uint16_t esc1Cmd = constrain(esc1Calculated, 1000, 1500); 
      uint16_t esc2Cmd = constrain(esc2Calculated, 1000, 1500); 

      // Send pulse outputs to ESCs
      esc1.writeMicroseconds(esc1Cmd);
      esc2.writeMicroseconds(esc2Cmd);

      // Servo Swashplate Mixing
      int s1_out = S1_NEUTRAL + (DIR_S1_ROLL * pidRoll) + (pidYaw * 0.4);
      servo1.write(constrain(s1_out, S1_MIN, S1_MAX));

      int s2_out = S2_NEUTRAL + (DIR_S2_PITCH * pidPitch);
      servo2.write(constrain(s2_out, S2_MIN, S2_MAX));

      int s3_out = S3_NEUTRAL + (DIR_S3_PITCH * pidPitch);
      servo3.write(constrain(s3_out, S3_MIN, S3_MAX));

      int s4_out = S4_NEUTRAL + (DIR_S4_ROLL * pidRoll) - (pidYaw * 0.4);
      servo4.write(constrain(s4_out, S4_MIN, S4_MAX));

    } else {
      // SAFE DISARMED STATE (Motors Disabled, Servos at Neutral)
      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
      servo1.write(S1_NEUTRAL);
      servo2.write(S2_NEUTRAL);
      servo3.write(S3_NEUTRAL);
      servo4.write(S4_NEUTRAL);
    }
  }
}