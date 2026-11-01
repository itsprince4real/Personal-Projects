#include <esp_now.h>
#include <WiFi.h>

// Right stick
const int PIN_VRX = 34;
const int PIN_VRY = 35;
const bool INVERT_ROLL  = true;
const bool INVERT_PITCH = false;

// Left stick (throttle)
const int PIN_THR = 33;                  // up/down axis
const bool THROTTLE_UP_IS_LOW = false; 
const int THROTTLE_MAX = 500;            // 500 = 50% of available range (0-1000)

int centerVRX = 2048;
int centerVRY = 2048;
int centerThr = 2048;

typedef struct __attribute__((packed)) struct_message {
  uint8_t magic;
  int8_t roll;
  int8_t pitch;
  uint16_t throttle;   // 0 - 500
} struct_message;

struct_message controlData;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);
  pinMode(PIN_VRX, INPUT);
  pinMode(PIN_VRY, INPUT);
  pinMode(PIN_THR, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed!");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Serial.println("Calibrating... DO NOT TOUCH EITHER STICK!");
  long sumX = 0, sumY = 0, sumT = 0;
  for (int i = 0; i < 100; i++) {
    sumX += analogRead(PIN_VRX);
    sumY += analogRead(PIN_VRY);
    sumT += analogRead(PIN_THR);
    delay(10);
  }
  centerVRX = sumX / 100;
  centerVRY = sumY / 100;
  centerThr = sumT / 100;
  Serial.printf("Center X=%d Y=%d T=%d\n", centerVRX, centerVRY, centerThr);
}

int readJoystick(int pin, int center) {
  int raw = analogRead(pin);
  int diff = raw - center;
  if (abs(diff) < 200) return 0;

  if (diff > 0) return map(raw, center + 200, 4000, 0, 30);
  else          return map(raw, 100, center - 200, -30, 0);
}

// Returns 0..THROTTLE_MAX. Idle and the "wrong" half of the stick give 0.
int readThrottle() {
  analogRead(PIN_THR);                       // dummy read to settle the ADC
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(PIN_THR);
  int raw = sum / 4;

  int diff = THROTTLE_UP_IS_LOW ? (centerThr - raw) : (raw - centerThr);
  if (diff < 200) return 0;                  // deadband / pulled the other way

  int span = THROTTLE_UP_IS_LOW ? (centerThr - 100) : (4000 - centerThr);
  int t = map(diff, 200, span, 0, THROTTLE_MAX);
  return constrain(t, 0, THROTTLE_MAX);
}

void loop() {
  int r = readJoystick(PIN_VRY, centerVRY);
  int p = readJoystick(PIN_VRX, centerVRX);
  r = constrain(r, -30, 30);
  p = constrain(p, -30, 30);

  controlData.magic    = 0xA5;
  controlData.roll     = INVERT_ROLL  ? -r : r;
  controlData.pitch    = INVERT_PITCH ? -p : p;
  controlData.throttle = readThrottle();

  esp_now_send(broadcastAddress, (uint8_t *)&controlData, sizeof(controlData));

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.printf("Roll: %d | Pitch: %d | Throttle: %d (%d%%)\n",
                  controlData.roll, controlData.pitch,
                  controlData.throttle, controlData.throttle / 10);
    lastPrint = millis();
  }
  delay(15);
}