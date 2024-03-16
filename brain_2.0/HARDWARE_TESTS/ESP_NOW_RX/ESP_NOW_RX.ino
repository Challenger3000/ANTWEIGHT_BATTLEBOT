// esp_now reciever 
#include <WiFi.h>
#include <esp_now.h>
uint8_t id = 1;
unsigned long last_packet=0;

typedef struct struct_message {
  byte mode;
  byte id  ;
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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  last_packet=millis();
  
  // Serial.print(myData.mode);
  // Serial.print("\t");
  // Serial.print(myData.id);
  // Serial.print("\t");
  // Serial.print(myData.ch01);
  // Serial.print("\t");
  // Serial.println(myData.ch02);
}

void init_esp_now(){
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

// esp_now

void setup(){
  Serial.begin(115200);  
  while(!Serial){
    ;
  }
  Serial.println("Starting...");
  init_esp_now();
}

void loop(){

}
