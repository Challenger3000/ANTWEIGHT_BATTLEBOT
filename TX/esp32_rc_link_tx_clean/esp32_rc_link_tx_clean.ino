
// esp now
#include <esp_now.h>
#include <WiFi.h>
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned long last_sendtime = 0;
esp_now_peer_info_t peerInfo;

// hardware pins
#define g_x 10
#define g_y 9

// variables
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
int ch1_offset = 0;
int ch2_offset = 0;
int motorA = 0;
int motorB = 0;
uint8_t expo_B = 0;



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

long mapWithMidpoint(long value, long fromLow, long fromMid, long fromHigh, long toLow, long toHigh) {
  // First, adjust the range around the midpoint
  if(value < fromMid) {
    // Map the lower half
    return map(value, fromLow, fromMid, toLow, toHigh/2);
  } else {
    // Map the upper half
    return map(value, fromMid, fromHigh, toHigh/2, toHigh);
  }
}


void calibrate(){
  Serial.println("Calibrating...");
  ch1_offset = 0;
  ch2_offset = 0;
  for(int i=0; i<500; i++){
    ch1_offset += analogRead(g_x);
    ch2_offset += analogRead(g_y);
    delay(1);
  }
  ch1_offset = ch1_offset/500;
  ch2_offset = ch2_offset/500;
  
  Serial.println("Calibration done");
}

uint8_t calculate_expo(int rc_in, float expo){

  float scaled_rc = ((float)rc_in/2047)-1;
  scaled_rc = constrain(scaled_rc, -1.0, 1.0);
  float expo_factor = scaled_rc * (1 - expo + (expo * scaled_rc * scaled_rc));
  uint8_t expo_val = (uint8_t)((expo_factor+1)*127.5);
  return expo_val;

}

void send_joysitck(){


  
  motorA = 255-mapWithMidpoint(constrain(analogRead(g_x),380,3780), 380, ch1_offset, 3780, 0, 255);   // big joystick
  motorB = motorA;
  expo_B = calculate_expo(mapWithMidpoint(constrain(analogRead(g_y),260,3870), 260, ch2_offset, 3870, 0, 4095), 0.2);
  
  motorA = constrain(motorA, 0, 255);
  motorB = constrain(motorB, 0, 255);

  motorA -= (expo_B-128)/4; //stock small joystick
  motorB += (expo_B-128)/4;
  

  // motorA -= (expo_B-128); // big small joystick 
  // motorB += (expo_B-128);

  motorA = constrain(motorA, 0, 255);
  motorB = constrain(motorB, 0, 255);





  // myData.mode = 1;
  // myData.id   = 1;
  // myData.ch01 = map(analogRead(g_x),380,3780,0,255);
  // myData.ch02 = map(analogRead(g_y),260,3870,0,255);
  // myData.ch01 = mapWithMidpoint(analogRead(g_x),380,ch1_offset,3780,0,255);
  // myData.ch02 = mapWithMidpoint(analogRead(g_y),260,ch2_offset,3870,0,255);
  myData.mode = 1;
  myData.id   = 1;
  myData.ch01 = motorA;
  myData.ch02 = motorB;
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
  
  Serial.print(myData.ch01);
  Serial.print("\t");
  Serial.println(myData.ch02);

  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
}

void setup() {
  Serial.begin(115200);
  pinMode(g_x, INPUT);
  pinMode(g_y, INPUT);
  delay(150);
  calibrate();
  init_data_structures();
  init_esp_now();
}

void loop() {
  send_joysitck();
  // Serial.print(analogRead(g_x));
  // Serial.print("\t");
  // Serial.println(analogRead(g_y));
  delay(10);
}
