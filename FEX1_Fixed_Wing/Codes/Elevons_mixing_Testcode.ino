#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// --- PIN DEFINITIONS ---
const int SERVO_L_PIN = 18;
const int SERVO_R_PIN = 19;
const int ESC_PIN     = 4;

Servo servoL;
Servo servoR;
Servo esc;

const int BASE_ANGLE = 105;
const int MAX_DEFLECTION = 30;

int currentL = BASE_ANGLE;
int currentR = BASE_ANGLE;
volatile int targetL = BASE_ANGLE;
volatile int targetR = BASE_ANGLE;

// Throttle (all in microseconds)
const int ESC_ARM_US  = 1000;   // used only for arming
const int ESC_IDLE_US = 1280;   // just BELOW the point where the motor starts (~1300)
const int ESC_MAX_US  = 1500;   // 50% of the 1000-2000 range
const int THROTTLE_MAX = 500;   // transmitter range 0..500
const unsigned long FAILSAFE_MS = 500;

volatile int targetUs = ESC_IDLE_US;
int currentUs = ESC_IDLE_US;
volatile unsigned long lastRxMs = 0;

unsigned long previousMillis = 0;
const int UPDATE_INTERVAL = 8;

typedef struct __attribute__((packed)) struct_message {
  uint8_t magic;
  int8_t roll;
  int8_t pitch;
  uint16_t throttle;
} struct_message;

struct_message controlData;

void OnDataRecv(const esp_now_recv_info *esp_now_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(controlData)) return;
  memcpy(&controlData, incomingData, sizeof(controlData));
  if (controlData.magic != 0xA5) return;

  lastRxMs = millis();

  int pitch = controlData.pitch;
  int roll  = controlData.roll;

  int calcL = BASE_ANGLE - pitch + roll;   // pin 18
  int calcR = BASE_ANGLE - pitch - roll;   // pin 19

  targetL = constrain(calcL, BASE_ANGLE - MAX_DEFLECTION, BASE_ANGLE + MAX_DEFLECTION);
  targetR = constrain(calcR, BASE_ANGLE - MAX_DEFLECTION, BASE_ANGLE + MAX_DEFLECTION);

  // 0..500 -> IDLE..MAX
  int t = constrain((int)controlData.throttle, 0, THROTTLE_MAX);
  targetUs = ESC_IDLE_US + (t * (ESC_MAX_US - ESC_IDLE_US)) / THROTTLE_MAX;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoL.setPeriodHertz(50);
  servoR.setPeriodHertz(50);
  esc.setPeriodHertz(50);

  servoL.attach(SERVO_L_PIN, 500, 2500);
  servoR.attach(SERVO_R_PIN, 500, 2500);
  esc.attach(ESC_PIN, 1000, 2000);

  servoL.write(currentL);
  servoR.write(currentR);

  // Arm the ESC: hold minimum throttle for 3 seconds
  esc.writeMicroseconds(ESC_ARM_US);
  Serial.println("Arming ESC...");
  delay(3000);

  // Move up to the idle level (just below motor start)
  esc.writeMicroseconds(ESC_IDLE_US);
  Serial.println("Idle set");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed!");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  lastRxMs = millis();
  Serial.println("Receiver Ready");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= UPDATE_INTERVAL) {
    previousMillis = currentMillis;
    bool moved = false;

    if (currentL != targetL) {
      int errorL = targetL - currentL;
      int stepL = errorL / 4;
      if (stepL == 0) stepL = (errorL > 0 ? 1 : -1);
      currentL += stepL;
      moved = true;
    }

    if (currentR != targetR) {
      int errorR = targetR - currentR;
      int stepR = errorR / 4;
      if (stepR == 0) stepR = (errorR > 0 ? 1 : -1);
      currentR += stepR;
      moved = true;
    }

    if (moved) {
      servoL.write(currentL);
      servoR.write(currentR);
    }

    // FAILSAFE: no valid packet for 500 ms -> back to idle (motor stopped), flaps centred
    if (currentMillis - lastRxMs > FAILSAFE_MS) {
      targetUs = ESC_IDLE_US;
      targetL = BASE_ANGLE;
      targetR = BASE_ANGLE;
    }

    // Throttle: gentle ramp up, quicker ramp down
    int tgt = targetUs;
    if (currentUs < tgt) {
      currentUs = min(currentUs + 4, tgt);
    } else if (currentUs > tgt) {
      currentUs = max(currentUs - 10, tgt);
    }
    esc.writeMicroseconds(currentUs);
  }
}