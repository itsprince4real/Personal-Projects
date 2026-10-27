#include <esp_now.h>
#include <WiFi.h>

// --- PIN MAP ---
const int PIN_ROLL     = 32; // Right Stick (Left/Right)
const int PIN_THROTTLE = 35; // Left Stick  (Up/Down)

// Calibrated resting centers and EMA Filters
int centerRoll = 2048, centerThrottle = 2048;
float filterRoll = 2048.0, filterThrottle = 2048.0;
const float EMA_ALPHA = 0.25; // Filter strength for smooth, noise-free reading

// Data packet (Roll + Throttle)
typedef struct struct_message {
  int8_t roll;       // -20 to +20 deg target
  uint16_t throttle; // 1000 us (Idle) to 1500 us (50% Max Power)
} struct_message;

struct_message controlData;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Read control stick with EMA Filter + Deadband
int readFilteredAxis(int pin, float &filteredVal, int centerVal, int maxAngle) {
  int raw = analogRead(pin);
  
  // Apply Exponential Moving Average (EMA) filter
  filteredVal = (EMA_ALPHA * raw) + ((1.0 - EMA_ALPHA) * filteredVal);
  float diff = filteredVal - centerVal;
  
  // Deadband around center to eliminate mechanical drift / residual noise
  if (abs(diff) < 120.0) return 0; 
  
  if (diff > 0) {
    return map((long)diff, 120, 4095 - centerVal, 0, maxAngle);
  } else {
    return map((long)diff, -120, -centerVal, 0, -maxAngle);
  }
}

void setup() {
  Serial.begin(115200);
  
  analogSetAttenuation(ADC_11db);
  pinMode(PIN_ROLL, INPUT);
  pinMode(PIN_THROTTLE, INPUT);

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

  // Automatic Center Calibration for both sticks
  Serial.println("Calibrating Sticks... DO NOT TOUCH!");
  long sumR = 0, sumT = 0;
  for (int i = 0; i < 100; i++) {
    sumR += analogRead(PIN_ROLL);
    sumT += analogRead(PIN_THROTTLE);
    delay(10);
  }
  centerRoll     = sumR / 100;
  centerThrottle = sumT / 100;

  filterRoll     = centerRoll;     // Initialize filter baseline
  filterThrottle = centerThrottle;
  
  Serial.println("Calibration Done!");
}

void loop() {
  // 1. Read steering input (mapped to max +-20 degrees)
  controlData.roll = readFilteredAxis(PIN_ROLL, filterRoll, centerRoll, 20);

  // 2. Read left joystick Y-axis for motor throttle (0% at center, 50% max UP)
  int rawThr = analogRead(PIN_THROTTLE);
  filterThrottle = (EMA_ALPHA * rawThr) + ((1.0 - EMA_ALPHA) * filterThrottle);

  if (filterThrottle > (centerThrottle + 120)) {
    // Upper half mapped strictly to 1000 us (0%) - 1500 us (50% max)
    controlData.throttle = map((long)filterThrottle, centerThrottle + 120, 4095, 1000, 1500);
  } else {
    controlData.throttle = 1000; // Center rest or pull DOWN = Motor Off
  }
  controlData.throttle = constrain(controlData.throttle, 1000, 1500);

  // Send Data via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *) &controlData, sizeof(controlData));

  // Debug Output
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.print("Roll Cmd: "); Serial.print(controlData.roll);
    Serial.print(" deg | Throttle: "); Serial.print(controlData.throttle);
    Serial.println(" us");
    lastPrint = millis();
  }

  delay(15); // ~66Hz update rate
}