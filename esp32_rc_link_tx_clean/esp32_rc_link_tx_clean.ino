
// esp now
#include <esp_now.h>
#include <WiFi.h>
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned long last_sendtime = 0;

// hardware pins
#define g_x 9
#define g_y 10

// data structures
typedef struct struct_message {
  byte mode;
  byte id;
  byte ch01;
  byte ch02;
  byte ch03;
  byte ch04;
  byte ch05;
  byte ch06;
  byte ch07;
  byte ch08;
  byte ch09;
  byte ch10;
  byte ch11;
  byte ch12;
  byte ch13;
  byte ch14;
  byte ch15;
  byte ch16;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

void init_data_structures(){
  myData.mode = 0;
  myData.id   = 0;
  myData.ch01 = 0;
  myData.ch02 = 0;
  myData.ch03 = 0;
  myData.ch04 = 0;
  myData.ch05 = 0;
  myData.ch06 = 0;
  myData.ch07 = 0;
  myData.ch08 = 0;
  myData.ch09 = 0;
  myData.ch10 = 0;
  myData.ch11 = 0;
  myData.ch12 = 0;
  myData.ch13 = 0;
  myData.ch14 = 0;
  myData.ch15 = 0;
  myData.ch16 = 0;
}

void init_esp_now(){
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void send_joysitck(){
  myData.mode = 1;
  myData.id   = 1;
  myData.ch01 = map(analogRead(g_x),0,4095,0,255);
  myData.ch02 = map(analogRead(g_y),0,4095,0,255);
  // myData.ch03 = mapWithMidpoint(analogRead(CH3),0,1799,4095,0,255);
  // myData.ch04 = mapWithMidpoint(analogRead(CH4),0,1799,4095,0,255);
  myData.ch03 = 50;
  myData.ch04 = 50;
  myData.ch05 = 50;
  myData.ch06 = 50;
  myData.ch07 = 50;
  myData.ch08 = 50;
  myData.ch09 = 50;
  myData.ch10 = 50;
  myData.ch11 = 50;
  myData.ch12 = 50;
  myData.ch13 = 50;
  myData.ch14 = 50;
  myData.ch15 = 50;
  myData.ch16 = 130;

  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
}

void setup() {
  Serial.begin(115200);
  pinMode(g_x, INPUT);
  pinMode(g_y, INPUT);
  init_data_structures();
  init_esp_now();
}

void loop() {
  send_joysitck();
  Serial.print(analogRead(g_x));
  Serial.print("\t");
  Serial.println(analogRead(g_y));
  delay(10);
}
