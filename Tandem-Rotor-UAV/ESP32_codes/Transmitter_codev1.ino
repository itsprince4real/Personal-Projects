#include <esp_now.h>
#include <WiFi.h>

const int THROTTLE_PIN = 34; // Update to your joystick pin

// Adjust this threshold to match your joystick's rest/center reading
const int REST_CENTER_RAW = 2100; 

typedef struct Packet {
  int throttle;
} Packet;

Packet controlData;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
  Serial.begin(115200);
  pinMode(THROTTLE_PIN, INPUT);

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
  Serial.println("Direct Positional Control Active (50us Steps).");
}

void loop() {
  int rawValue = analogRead(THROTTLE_PIN);
  int outputThrottle = 950; // Default zero throttle at rest position

  // If stick is pushed UP past rest center position
  if (rawValue > REST_CENTER_RAW) {
    // Map raw stick position directly to motor pulse range (1150us to 1700us)
    int mappedThrottle = map(rawValue, REST_CENTER_RAW, 4095, 1150, 1700);
    
    // Snap directly to nearest 50us step (1150, 1200, 1250, 1300, etc.)
    outputThrottle = (mappedThrottle / 50) * 50;
  }

  // Constrain bounds
  outputThrottle = constrain(outputThrottle, 950, 1700);
  controlData.throttle = outputThrottle;

  // Transmit immediately
  esp_now_send(broadcastAddress, (uint8_t *) &controlData, sizeof(controlData));

  // Serial Monitor debug readout
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.print("Raw Stick Pos: ");
    Serial.print(rawValue);
    Serial.print(" | Direct Throttle Output: ");
    Serial.print(controlData.throttle);
    Serial.println("us");
    lastPrint = millis();
  }

  delay(20); // 50Hz RC update rate
}