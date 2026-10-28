#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>

// --- PIN DEFINITIONS ---
const int SERVO1_PIN = 18; 
const int SERVO2_PIN = 19; 
const int SERVO_CENTER1 = 74; 
const int SERVO_CENTER2 = 79; 

const int ESC1_PIN = 4; 
const int ESC2_PIN = 5; 

const int MPU_ADDR = 0x68; // MPU6500 / MPU6050 I2C address

Servo servo1;
Servo servo2;
Servo esc1;
Servo esc2;

// --- MPU6500 & SENSOR FUSION VARIABLES ---
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float rollAngle = 0, pitchAngle = 0;
unsigned long prevTime = 0;

// --- PID TUNING PARAMETERS ---
// Roll PID (Servos auto-leveling)
float Kp_roll = 0.35;
float Ki_roll = 0.01;
float Kd_roll = 0.05;
float errorRoll = 0, lastErrorRoll = 0, integralRoll = 0;

// Yaw PID (Differential Motor Throttle stabilization)
float Kp_yaw = 1.2;
float Kd_yaw = 0.1;
float errorYaw = 0, lastErrorYaw = 0;

// Data packet (matching Transmitter)
typedef struct struct_message {
  int8_t roll;       // -20 to +20 deg
  uint16_t throttle; // 1000 to 1500 us
} struct_message;

struct_message incomingData;

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
}

// --- MPU6500 FUNCTIONS ---
void initMPU() {
  Wire.begin(21, 22); // SDA = 21, SCL = 22
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Wake up MPU
  Wire.endTransmission(true);

  // Calibrate Gyro offset at rest
  Serial.println("Calibrating MPU6500 Gyro... Keep still!");
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // Gyro measurements register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    
    gyroX_offset += (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    gyroY_offset += (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    gyroZ_offset += (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    delay(5);
  }
  gyroX_offset /= 200.0;
  gyroY_offset /= 200.0;
  gyroZ_offset /= 200.0;
  Serial.println("MPU6500 Calibrated!");
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Accel & Gyro start register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t rawAccX = Wire.read() << 8 | Wire.read();
  int16_t rawAccY = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

  // Scale raw values (Accel ±2g, Gyro ±250 deg/s)
  accX = rawAccX / 16384.0;
  accY = rawAccY / 16384.0;
  accZ = rawAccZ / 16384.0;

  gyroX = (rawGyroX / 131.0) - gyroX_offset;
  gyroY = (rawGyroY / 131.0) - gyroY_offset;
  gyroZ = (rawGyroZ / 131.0) - gyroZ_offset;

  // Calculate Delta Time (dt)
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;

  // Accelerometer Roll/Pitch angles
  float accRollAngle = atan2(accY, accZ) * 57.2958;

  // Complementary Filter (96% Gyro integration + 4% Accel absolute angle)
  rollAngle = 0.96 * (rollAngle + gyroX * dt) + 0.04 * accRollAngle;
}

void setup() {
  Serial.begin(115200);

  // ESP32 Servo Timer allocation
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2500);
  servo2.attach(SERVO2_PIN, 500, 2500);

  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);

  // Arming sequence outputs
  servo1.write(SERVO_CENTER1);
  servo2.write(SERVO_CENTER2);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);

  // Init MPU6500
  initMPU();

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed!");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  prevTime = micros();
  Serial.println("Receiver Ready with Flight Stabilization!");
}

void loop() {
  // Update IMU measurements
  readMPU();

  // --- 1. SERVO AUTO-LEVELING (MAX ±5 DEGREES IMU INFLUENCE) ---
  int pilotRollCmd = constrain(incomingData.roll, -20, 20);
  
  // Target angle is stick command; Error is difference between target and current craft roll
  errorRoll = pilotRollCmd - rollAngle;
  integralRoll = constrain(integralRoll + (errorRoll * 0.01), -10.0, 10.0);
  float derivativeRoll = errorRoll - lastErrorRoll;
  lastErrorRoll = errorRoll;

  float pidRollOutput = (Kp_roll * errorRoll) + (Ki_roll * integralRoll) + (Kd_roll * derivativeRoll);
  
  // Hard limit IMU correction contribution to MAX ±5 degrees as requested
  float imuServoCorrection = constrain(pidRollOutput, -5.0, 5.0);

  int targetServoPos1 = SERVO_CENTER1 + pilotRollCmd + (int)imuServoCorrection;
  int targetServoPos2 = SERVO_CENTER2 - pilotRollCmd - (int)imuServoCorrection;

  servo1.write(constrain(targetServoPos1, 0, 180));
  servo2.write(constrain(targetServoPos2, 0, 180));

  // --- 2. MOTOR YAW STABILIZATION (DIFFERENTIAL THROTTLE) ---
  uint16_t baseThrottle = constrain(incomingData.throttle, 1000, 1600);
  
  uint16_t motor1Cmd = 1000;
  uint16_t motor2Cmd = 1000;

  // Only apply differential stabilization if motors are active above idle
  if (baseThrottle > 1050) {
    errorYaw = 0.0 - gyroZ; // Target yaw rate = 0 deg/s
    float derivativeYaw = errorYaw - lastErrorYaw;
    lastErrorYaw = errorYaw;

    float yawCorrection = (Kp_yaw * errorYaw) + (Kd_yaw * derivativeYaw);
    
    // Differential motor mix (limits correction to ±100us)
    yawCorrection = constrain(yawCorrection, -100.0, 100.0);

    motor1Cmd = constrain((int)baseThrottle + (int)yawCorrection, 1000, 1600);
    motor2Cmd = constrain((int)baseThrottle - (int)yawCorrection, 1000, 1600);
  } else {
    // Reset integral terms when on the ground / idle
    integralRoll = 0;
  }

  esc1.writeMicroseconds(motor1Cmd);
  esc2.writeMicroseconds(motor2Cmd);

  // Debug Output
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.print("IMU Roll: "); Serial.print(rollAngle, 1);
    Serial.print("° | S1: "); Serial.print(targetServoPos1);
    Serial.print("° | S2: "); Serial.print(targetServoPos2);
    Serial.print("° | M1: "); Serial.print(motor1Cmd);
    Serial.print("us | M2: "); Serial.print(motor2Cmd);
    Serial.println("us");
    lastPrint = millis();
  }

  delay(5); // ~200Hz loop execution rate
}