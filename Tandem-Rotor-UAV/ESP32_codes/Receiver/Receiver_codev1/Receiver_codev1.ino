#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

Servo escFront; // GPIO 25
Servo escRear;  // GPIO 26

typedef struct Packet {
  int throttle;
} Packet;

Packet controlData;
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 500;

// Receive callback for ESP32 Core v3.x
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&controlData, incomingData, sizeof(controlData));
  lastPacketTime = millis();

  // Debug Print on Data Arrival
  Serial.print("RX Received Packet -> Throttle: ");
  Serial.print(controlData.throttle);
  Serial.println("us");
}

void setup() {
  Serial.begin(115200);

  escFront.attach(25, 900, 2000);
  escRear.attach(26, 900, 2000);
  escFront.writeMicroseconds(950);
  escRear.writeMicroseconds(950);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Receiver Active. Listening for packets...");
}

void loop() {
  if (millis() - lastPacketTime > FAILSAFE_TIMEOUT) {
    escFront.writeMicroseconds(950);
    escRear.writeMicroseconds(950);
    
    // Print warning periodically during loss of signal
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn > 1000) {
      Serial.println("WARNING: Signal Lost! Failsafe Active (950us)");
      lastWarn = millis();
    }
  } else {
    escFront.writeMicroseconds(controlData.throttle);
    escRear.writeMicroseconds(controlData.throttle);
  }
  
  delay(10);
}