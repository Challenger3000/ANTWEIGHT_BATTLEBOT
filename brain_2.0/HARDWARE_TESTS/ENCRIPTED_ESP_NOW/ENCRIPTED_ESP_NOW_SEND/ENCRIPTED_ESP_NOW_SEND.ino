
#include "esp_wifi.h"
#include <esp_now.h>
#include <WiFi.h>

// DC:DA:0C:41:E9:4C RX
// DC:DA:0C:40:DA:DC TX

// REPLACE WITH THE RECEIVER'S MAC Address
// 0xDC, 0xDA, 0x0C, 0x40, 0xDA, 0xDC

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t receiverAddress[] = { 0xDC, 0xDA, 0x0C, 0x41, 0xE9, 0x4C};
// uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// PMK and LMK keys
static const char* PMK_KEY_STR = "00XXmkwei/lpPÇf";
static const char* LMK_KEY_STR = "00XXmkwei/lpPÇf";

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int counter;
    int x;
    int y;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Counter variable to keep track of number of sent packets
int counter;

// // callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   Serial.print("\r\nLast Packet Send Status:\t");
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
// }
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  {
    delay(100);
  }  
  Serial.println("Starting");
  
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  esp_wifi_set_channel(14,WIFI_SECOND_CHAN_NONE);
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("There was an error initializing ESP-NOW");
    return;
  }
  
  // Set PMK key
  esp_now_set_pmk((uint8_t *)PMK_KEY_STR);
  
  // Register the receiver board as peer
  esp_now_peer_info_t peerInfo;  
  memset(&peerInfo, 0, sizeof(peerInfo));

  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 14;
  //Set the receiver device LMK key
  for (uint8_t i = 0; i < 16; i++) {
    peerInfo.lmk[i] = LMK_KEY_STR[i];
  }
  // Set encryption to true
  // peerInfo.encrypt = true;
  
  // Add receiver as peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of transmitted packet
  // esp_now_register_send_cb(OnDataSent);
}
void loop() {
  static unsigned long lastEventTime = millis();
  static const unsigned long EVENT_INTERVAL_MS = 20;
  if ((millis() - lastEventTime) > EVENT_INTERVAL_MS) {
    lastEventTime = millis();
    
    // Set values to send
    myData.counter = counter++;
    myData.x = random(0,50);
    myData.y = random(0,50);
  
    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    Serial.println("Sent");
  }
}