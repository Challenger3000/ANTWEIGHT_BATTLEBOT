/*
   -- New project --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.11 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.11.4 or later version;
     - for iOS 1.9.1 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG  

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_WIFI_POINT
#include <WiFi.h>

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // original
  { 255,4,0,4,0,52,0,16,31,1,5,32,32,58,24,24,2,26,31,10,
  48,10,64,13,13,1,26,31,79,78,0,31,79,70,70,0,4,128,10,84,
  45,8,2,26,68,49,0,0,63,48,8,36,86,95,99,101,108,108,0 };

// uint8_t RemoteXY_CONF[] =   // 54 bytes
//   { 255,5,0,0,0,47,0,16,31,1,1,1,20,65,22,20,202,31,0,1,
//   1,42,56,21,21,202,31,0,1,1,0,56,20,20,202,31,0,1,1,20,
//   44,22,21,202,31,0,4,128,8,20,47,13,2,26 };

// struct {

//     // input variables
//   uint8_t b5; // =1 if button pressed, else =0 
//   uint8_t b6; // =1 if button pressed, else =0 
//   uint8_t b4; // =1 if button pressed, else =0 
//   uint8_t b2; // =1 if button pressed, else =0 
//   int8_t s1; // =0..100 slider position 

//     // other variable
//   uint8_t connect_flag;  // =1 if wire connected, else =0 

// } RemoteXY;

struct {

    // input variables
  int8_t j1_x; // from -100 to 100  
  int8_t j1_y; // from -100 to 100  
  uint8_t t1; // =1 if state is ON, else =0 
  int8_t s1; // =0..100 slider position 

    // output variables
  float g1;

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#include <WiFi.h>
#include <esp_now.h>

// eeprom
#include <EEPROM.h>
#define EEPROM_SIZE 10

#define pwm_A 27
#define INA_1 12  
#define INA_2 14

#define pwm_B 26
#define INB_1 4
#define INB_2 2

#define vbat 34

#define button 23

// timing
unsigned long last_loop_update = 0;
unsigned int loop_hz = 100;
unsigned int loop_ms = 1000/loop_hz;

// logic
uint8_t id = 1;
bool rx_type_wifi = false;

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

unsigned long last_packet=0;

// servo
const int servoPin = 21;
unsigned long servo_start_time = 0;
unsigned long servo_off_time = 0;
unsigned long servo__next_on_time = 0;
const int servoPeriod = 20000;
uint8_t current_duty = 0;
bool servo_on = true;

void init_servo_pwm() {
  servo_start_time = micros();
  digitalWrite(servoPin, HIGH);
  servo_on = true;
  servo_off_time = servo_start_time + map(current_duty, 0, 255, 1000, 2000);
  servo__next_on_time = servo_start_time + servoPeriod;
}

void update_servo() {
  unsigned long currentTime = micros();
  if(servo_on){
    if (currentTime >= servo_off_time) {
      digitalWrite(servoPin, LOW);
      digitalWrite(22, LOW);
      servo_on = false;
    }
  }else{
    if (currentTime >= servo__next_on_time) {
      digitalWrite(servoPin, HIGH);
      digitalWrite(22, HIGH);
      servo_on = true;
      servo_off_time = servo__next_on_time + map(current_duty, 0, 255, 1000, 2000);
      servo__next_on_time += servoPeriod;
    }
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  last_packet=millis();
  

  // Serial.print(myData.mode);
  // Serial.print("\t");
  // Serial.print(myData.id);
  // Serial.print("\t");
  // Serial.print(myData.ch01);
  // Serial.print("\t");
  // Serial.print(myData.ch02);
  // Serial.print("\t");
  // Serial.print(myData.ch03);
  // Serial.print("\t");
  // Serial.print(myData.ch04);
  // Serial.print("\t");
  // Serial.print(myData.ch05);
  // Serial.print("\t");
  // Serial.print(myData.ch06);
  // Serial.print("\t");
  // Serial.print(myData.ch07);
  // Serial.print("\t");
  // Serial.print(myData.ch08);
  // Serial.print("\t");
  // Serial.print(myData.ch09);
  // Serial.print("\t");
  // Serial.print(myData.ch10);
  // Serial.print("\t");
  // Serial.print(myData.ch11);
  // Serial.print("\t");
  // Serial.print(myData.ch12);
  // Serial.print("\t");
  // Serial.print(myData.ch13);
  // Serial.print("\t");
  // Serial.print(myData.ch14);
  // Serial.print("\t");
  // Serial.print(myData.ch15);
  // Serial.print("\t");
  // Serial.println(myData.ch16);
}


const int ledPin1 = 27; //a
const int ledPin2 = 26; //b
// const int ledPin3 = 21; //esc

const int freq_DRVR = 5000;
const int resolution_DRVR = 8;
// the number of the LED pin

const int ledChannel1 = 1; // Use channel 0 for ledPin1
const int ledChannel2 = 2; // Use channel 1 for ledPin2

void drive_motor_A(uint8_t speed){  
  if(speed>128){
    digitalWrite(INA_1, HIGH);
    digitalWrite(INA_2, LOW);
    ledcWrite(ledChannel1, ((speed-128)*2)+1);
    // Serial.print("A for ward: ");
    // Serial.println(((speed-128)*2)+1);
  }else if(speed==128){
    digitalWrite(INA_1, LOW);
    digitalWrite(INA_2, LOW);
    ledcWrite(ledChannel1, 0);
    // Serial.println("A STOP");
  }else if(speed<128){
    digitalWrite(INA_1, LOW);
    digitalWrite(INA_2, HIGH);
    ledcWrite(ledChannel1, ((128-speed)*2)-1);
    // Serial.print("A backward: ");
    // Serial.println(((128-speed)*2)-1);
  }
}

void drive_motor_B(uint8_t speed){
  if(speed>128){
    digitalWrite(INB_1, HIGH);
    digitalWrite(INB_2, LOW);
    ledcWrite(ledChannel2, ((speed-128)*2)+1);
    // Serial.print("B for ward: ");
    // Serial.println(((speed-128)*2)+1);
  }else if(speed==128){
    digitalWrite(INB_1, LOW);
    digitalWrite(INB_2, LOW);
    ledcWrite(ledChannel2, 0);
    // Serial.println("B STOP");
  }else if(speed<128){
    digitalWrite(INB_1, LOW);
    digitalWrite(INB_2, HIGH);
    ledcWrite(ledChannel2, ((128-speed)*2)-1);
    // Serial.print("B backward: ");
    // Serial.println(((128-speed)*2)-1);
  }
}

void setup() {  
  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(115200);
  pinMode(22, OUTPUT);

  pinMode(INA_1, OUTPUT);
  pinMode(INA_2, OUTPUT);
  pinMode(INB_1, OUTPUT);
  pinMode(INB_2, OUTPUT);
  pinMode(servoPin, OUTPUT);
  pinMode(vbat, INPUT);
  pinMode(button, INPUT);

  digitalWrite(INA_1, HIGH);
  digitalWrite(INA_2, LOW);
  digitalWrite(INB_1, HIGH);
  digitalWrite(INB_2, LOW);

  digitalWrite(22, HIGH);

  id = EEPROM.read(0);

  ledcSetup(ledChannel1, freq_DRVR, resolution_DRVR);
  ledcAttachPin(ledPin1, ledChannel1);

  ledcSetup(ledChannel2, freq_DRVR, resolution_DRVR);
  ledcAttachPin(ledPin2, ledChannel2);
  
  
  if(!digitalRead(button)){
    rx_type_wifi = true;
    RemoteXY_Init(); 
  }else{
    rx_type_wifi = false;
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != 0) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_register_recv_cb(OnDataRecv);
  }

  init_servo_pwm();
}

void loop() {
    update_servo();
  if(millis()-last_loop_update>=loop_ms){
    last_loop_update=millis();

    if(!rx_type_wifi){
      if(!digitalRead(button)){
        id++;
        if(id>4){
          id=1;
        }
        delay(500);
        digitalWrite(22, LOW);
        for(uint8_t i=0; i < id; i++){
          digitalWrite(22, HIGH);
          delay(200);
          digitalWrite(22, LOW);
          delay(200);
        }
        EEPROM.write(0, id);
        EEPROM.commit();
        
        Serial.print("NEW_ID: ");
        Serial.println(id);
        delay(200);
      }
      // if no packets for 100ms assume FS_RC
      // if( (millis()-last_packet) > 50 || id!=myData.ch16-127){
      if( (millis()-last_packet) > 50 ){
        digitalWrite(INA_1, LOW);
        digitalWrite(INA_2, LOW);
        digitalWrite(INB_1, LOW);
        digitalWrite(INB_2, LOW);
        ledcWrite(ledChannel1, 0);
        ledcWrite(ledChannel2, 0);

        current_duty = 0;
        Serial.print("NO SIGNAL my ch: ");
        Serial.print(id);
        Serial.print(" received id: ");
        Serial.println(myData.ch16-127);
      }else if(id==myData.ch16-127){
          if(myData.ch01>128){
          digitalWrite(INA_1, HIGH);
          digitalWrite(INA_2, LOW);
          ledcWrite(ledChannel1, ((myData.ch01-128)*2)+1);
          Serial.print("A for ward: ");
          Serial.println(((myData.ch01-128)*2)+1);
        }else if(myData.ch01==128){
          digitalWrite(INA_1, LOW);
          digitalWrite(INA_2, LOW);
          ledcWrite(ledChannel1, 0);
          Serial.println("A STOP");
        }else if(myData.ch01<128){
          digitalWrite(INA_1, LOW);
          digitalWrite(INA_2, HIGH);
          ledcWrite(ledChannel1, ((128-myData.ch01)*2)-1);
          Serial.print("A backward: ");
          Serial.println(((128-myData.ch01)*2)-1);
        }

        if(myData.ch02>128){
          digitalWrite(INB_1, HIGH);
          digitalWrite(INB_2, LOW);
          ledcWrite(ledChannel2, ((myData.ch02-128)*2)+1);
          Serial.print("B for ward: ");
          Serial.println(((myData.ch02-128)*2)+1);
        }else if(myData.ch02==128){
          digitalWrite(INB_1, LOW);
          digitalWrite(INB_2, LOW);
          ledcWrite(ledChannel2, 0);
          Serial.println("B STOP");
        }else if(myData.ch02<128){
          digitalWrite(INB_1, LOW);
          digitalWrite(INB_2, HIGH);
          ledcWrite(ledChannel2, ((128-myData.ch02)*2)-1);
          Serial.print("B backward: ");
          Serial.println(((128-myData.ch02)*2)-1);
        }
        
        current_duty = myData.ch03;
        Serial.print("SERVO: ");
        Serial.println(myData.ch03);
        // Serial.println(map(myData.ch03,0,255,0,180));
      }
    }else{
      RemoteXY_Handler();

      if(RemoteXY.t1){
      // if(true){
        int speed_A = map(RemoteXY.j1_y,-100,100,0,255)-round((float)RemoteXY.j1_x*0.5);
        int speed_B = map(RemoteXY.j1_y,-100,100,0,255)+round((float)RemoteXY.j1_x*0.5);
        Serial.print("SPEED_A ");
        Serial.println(speed_A);
        Serial.print("SPEED_B ");
        Serial.println(speed_B);
        if(speed_A>255) speed_A=255;
        if(speed_A<0)   speed_A=0;

        if(speed_B>255) speed_B=255;
        if(speed_B<0)   speed_B=0;

        // if(RemoteXY.b2 && RemoteXY.b4){         // forward-left
        //   drive_motor_A(round((float)(0-128   )*(float)((float)RemoteXY.s1/100.0))+128);
        //   drive_motor_B(round((float)(73-128  )*(float)((float)RemoteXY.s1/100.0))+128);
        // }else if(RemoteXY.b2 && RemoteXY.b6){   // fowrard-right
        //   drive_motor_A(round((float)(73-128  )*(float)((float)RemoteXY.s1/100.0))+128);
        //   drive_motor_B(round((float)(0-128   )*(float)((float)RemoteXY.s1/100.0))+128);
        // }else if(RemoteXY.b5 && RemoteXY.b4){   // backward-left
        //   drive_motor_A(round((float)(255-128 )*(float)((float)RemoteXY.s1/100.0))+128);
        //   drive_motor_B(round((float)(183-128 )*(float)((float)RemoteXY.s1/100.0))+128);
        // }else if(RemoteXY.b5 && RemoteXY.b6){   // backward-right
        //   drive_motor_A(round((float)(183-128 )*(float)((float)RemoteXY.s1/100.0))+128);
        //   drive_motor_B(round((float)(255-128 )*(float)((float)RemoteXY.s1/100.0))+128);
        // }else if(RemoteXY.b2){                  // forward
        //   drive_motor_A(round((float)(0-128   )*(float)((float)RemoteXY.s1/100.0))+128);
        //   drive_motor_B(round((float)(0-128   )*(float)((float)RemoteXY.s1/100.0))+128);
        // }else if(RemoteXY.b5){                  // backward
        //   drive_motor_A(round((float)(255-128 )*(float)((float)RemoteXY.s1/100.0))+128);
        //   drive_motor_B(round((float)(255-128 )*(float)((float)RemoteXY.s1/100.0))+128);
        // }else if(RemoteXY.b4){                  // left
        //   drive_motor_A(round((float)(39-128  )*(float)((float)RemoteXY.s1/100.0))+128);
        //   drive_motor_B(round((float)(217-128 )*(float)((float)RemoteXY.s1/100.0))+128);
        // }else if(RemoteXY.b6){                  // right
        //   drive_motor_A(round((float)(217-128 )*(float)((float)RemoteXY.s1/100.0))+128);
        //   drive_motor_B(round((float)(39-128  )*(float)((float)RemoteXY.s1/100.0))+128);
        // }else{
        //   drive_motor_A(128);
        //   drive_motor_B(128);
        // }
        // Serial.print("SPEED_A ");
        // Serial.print(speed_A);
        // Serial.print("\t\tSPEED_B ");
        // Serial.println(speed_B);
        
      }else{
        drive_motor_A(128);
        drive_motor_B(128);
      }


      // RemoteXY.g1 = (analogRead(vbat) / 199.34) / 3;
      // current_duty = map(RemoteXY.s1,-100,100,0,255);
      // Serial.print("SERVO: ");
      // Serial.println(map(RemoteXY.s1,0,100,0,255));
    }
  }
}
