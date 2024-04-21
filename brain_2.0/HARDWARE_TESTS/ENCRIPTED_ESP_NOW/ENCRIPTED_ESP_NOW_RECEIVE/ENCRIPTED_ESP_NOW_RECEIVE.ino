
#include "esp_wifi.h"
#include <esp_now.h>
#include <WiFi.h>




// DC:DA:0C:40:DA:DC    TX
// DC:DA:0C:41:E9:4C    RX

uint8_t masterMacAddress[] = {0xDC, 0xDA, 0x0C, 0x40, 0xDA, 0xDC};

// rssi
typedef struct {
  unsigned frame_ctrl: 16;
  unsigned duration_id: 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl: 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

int rssi_last;

// PMK and LMK keys
static const char* PMK_KEY_STR = "00XXmkwei/lpPÇf";
static const char* LMK_KEY_STR = "00XXmkwei/lpPÇf";

// Structure example to send data
// Must match the sender structure
typedef struct struct_message {
    int counter; // must be unique for each sender board
    int x;
    int y;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Function to print MAC address on Serial Monitor
void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
}

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
 
  Serial.print("Packet received from: ");
  printMAC(mac_addr);
  
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Packet number: ");
  Serial.println(myData.counter);
  Serial.print("X: ");
  Serial.println(myData.x);
  Serial.print("Y: ");
  Serial.println(myData.y);
  
  Serial.print("RSSI: ");
  Serial.println(rssi_last);
}

void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  int rssi = ppkt->rx_ctrl.rssi;
  rssi_last = rssi;
}

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
  
  // Set the PMK key
  esp_now_set_pmk((uint8_t *)PMK_KEY_STR);
  
  // Register the master as peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, masterMacAddress, 6);
  peerInfo.channel = 14;
  // Setting the master device LMK key
  for (uint8_t i = 0; i < 16; i++) {
    peerInfo.lmk[i] = LMK_KEY_STR[i];
  }
  // Set encryption to true
  peerInfo.encrypt = false;
  
     
  // if (esp_now_add_peer(&peerInfo) != ESP_OK){
  //   Serial.println("Failed to add peer");
  //   return;
  // }


  esp_now_register_recv_cb(OnDataRecv);
  
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
}
void loop() {
  
}