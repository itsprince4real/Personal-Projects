#include <esp_now.h>
#include <WiFi.h>

// --- SELECT THROTTLE BEHAVIOR FOR SPRING-LOADED STICK ---
// 0 = UPPER_HALF (Center rest = Idle/0% throttle, UP = 0-100% throttle)
// 1 = NUDGE_HOLD (Center rest = Hold throttle level, UP = Ramp up, DOWN = Ramp down)
#define THROTTLE_MODE 0 

// --- YOUR SPECIFIC PIN MAP (ALL ADC1) ---
const int PIN_THROTTLE = 35; // Left Y
const int PIN_YAW      = 34; // Left X
const int PIN_PITCH    = 33; // Right Y
const int PIN_ROLL     = 32; // Right X

// Calibrated resting centers
int centerPitch = 2048, centerRoll = 2048, centerYaw = 2048, centerThrottle = 2048;

// Persistent throttle memory for Nudge/Hold mode
float holdThrottle = 1000.0;

typedef struct struct_message {
  uint16_t throttle; // 1000 - 2000 us
  int8_t pitch;      // -30 to +30 deg target
  int8_t roll;       // -30 to +30 deg target
  int8_t yaw;        // -30 to +30 deg target
  bool armed;        // Safety switch
} struct_message;

struct_message controlData;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Read control stick relative to calibrated center baseline
int readCalibratedAxis(int pin, int centerVal, int maxAngle) {
  int raw = analogRead(pin);
  int diff = raw - centerVal;
  
  // Deadband around center to eliminate mechanical drift Jitter
  if (abs(diff) < 180) return 0; 
  
  if (diff > 0) {
    return map(diff, 180, 4095 - centerVal, 0, maxAngle);
  } else {
    return map(diff, -180, -centerVal, 0, -maxAngle);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Set ADC Attenuation for full 0-3.3V range
  analogSetAttenuation(ADC_11db);

  pinMode(PIN_THROTTLE, INPUT);
  pinMode(PIN_YAW, INPUT);
  pinMode(PIN_PITCH, INPUT);
  pinMode(PIN_ROLL, INPUT);

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

  // --- AUTOMATIC CENTER CALIBRATION ON BOOT ---
  Serial.println("Calibrating Center Points... DO NOT TOUCH STICKS!");
  long sumP = 0, sumR = 0, sumY = 0, sumT = 0;
  for (int i = 0; i < 60; i++) {
    sumT += analogRead(PIN_THROTTLE);
    sumY += analogRead(PIN_YAW);
    sumP += analogRead(PIN_PITCH);
    sumR += analogRead(PIN_ROLL);
    delay(20);
  }
  centerThrottle = sumT / 60;
  centerYaw      = sumY / 60;
  centerPitch    = sumP / 60;
  centerRoll     = sumR / 60;
  
  Serial.println("Calibration Done!");
  Serial.print("Centers -> Thr: "); Serial.print(centerThrottle);
  Serial.print(" | Yaw: "); Serial.print(centerYaw);
  Serial.print(" | Pitch: "); Serial.print(centerPitch);
  Serial.print(" | Roll: "); Serial.println(centerRoll);
}

void loop() {
  int rawThrottle = analogRead(PIN_THROTTLE);

  #if THROTTLE_MODE == 0
    // === MODE 0: UPPER HALF DIRECT CONTROL ===
    // Center Rest Position = 1000us (Idle). Pushing UP = 1000us to 2000us.
    if (rawThrottle > (centerThrottle + 180)) {
      controlData.throttle = map(rawThrottle, centerThrottle + 180, 4095, 1000, 2000);
    } else {
      controlData.throttle = 1000;
    }
  #else
    // === MODE 1: NUDGE & HOLD CONTROL ===
    // Resting at center maintains throttle. Pushing UP ramps throttle higher.
    // Pulling DOWN ramps throttle lower. Full DOWN = Hard Throttle Cut.
    int diffThr = rawThrottle - centerThrottle;
    if (abs(diffThr) > 200) {
      if (diffThr > 0) {
        holdThrottle += (diffThr / 200.0); // Ramp Up Rate
      } else {
        holdThrottle += (diffThr / 200.0); // Ramp Down Rate
      }
    }
    // Hard Cutoff when stick is pulled all the way DOWN
    if (rawThrottle < 300) {
      holdThrottle = 1000.0;
    }
    controlData.throttle = (uint16_t)constrain(holdThrottle, 1000.0, 2000.0);
  #endif

  controlData.throttle = constrain(controlData.throttle, 1000, 2000);

  // Read Steering Commands
  controlData.pitch = readCalibratedAxis(PIN_PITCH, centerPitch, 30);
  controlData.roll  = readCalibratedAxis(PIN_ROLL, centerRoll, 30);
  controlData.yaw   = readCalibratedAxis(PIN_YAW, centerYaw, 30);

  // Auto-Arming
  controlData.armed = (controlData.throttle > 1050);

  // Send Data
  esp_now_send(broadcastAddress, (uint8_t *) &controlData, sizeof(controlData));

  // Debug Output
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 150) {
    Serial.print("Thr: "); Serial.print(controlData.throttle);
    Serial.print("us | Pitch: "); Serial.print(controlData.pitch);
    Serial.print("deg | Roll: "); Serial.print(controlData.roll);
    Serial.print("deg | Yaw: "); Serial.print(controlData.yaw);
    Serial.print("deg | Armed: "); Serial.println(controlData.armed ? "YES" : "NO");
    lastPrint = millis();
  }

  delay(20); // 50Hz Transmission
}