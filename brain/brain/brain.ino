#include <WiFi.h>
#include <esp_now.h>

// eeprom
#include <EEPROM.h>
#define EEPROM_SIZE 10


// servo
#include <Servo.h>
static const int servoPin = 21;
Servo servo1;




#define pwm_A 27
#define INA_1 12
#define INA_2 14

#define pwm_B 26
#define INB_1 4
#define INB_2 2
#define INB_3 0

#define button 23


uint8_t id = 1;


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


// Callback function that will be executed when data is received
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

const int ledChannel = 0;
const int ledChanne2 = 1;
const int ledChanne3 = 2;
const int ledChanne4 = 3;
const int ledChanne5 = 4;

// const int ledChanne3 = 2;

// setting PWM properties
const int freq = 5000;
const int resolution = 8;
// the number of the LED pin

void setup() {
  
  EEPROM.begin(EEPROM_SIZE);


  Serial.begin(115200);
  pinMode(22, OUTPUT);

  pinMode(INA_1, OUTPUT);
  pinMode(INA_2, OUTPUT);
  // pinMode(pwm_A, OUTPUT);


  pinMode(INB_1, OUTPUT);
  pinMode(INB_2, OUTPUT);
  pinMode(INB_3, OUTPUT);
  
  pinMode(button, INPUT);

  digitalWrite(INA_1, HIGH);
  digitalWrite(INA_2, LOW);
  // analogWrite(pwm_A, 25);

  digitalWrite(INB_1, HIGH);
  digitalWrite(INB_2, LOW);
  // digitalWrite(INB_3, LOW);
  // analogWrite(pwm_B, 25);

  digitalWrite(22, HIGH);

  id = EEPROM.read(0);


  ledcSetup(1, freq, resolution);
  ledcAttachPin(ledPin1, 1);

  ledcSetup(3, freq, resolution);
  ledcAttachPin(ledPin2, 3);
  
  

  

  delay(500);


  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }  

  // esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  // esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_recv_cb(OnDataRecv);
  

}

void loop() {

    if(!digitalRead(button)){
      id++;
      if(id>4){
        id=1;
      }

      for(uint8_t i=0; i < id; i++){
        digitalWrite(22, LOW);
        delay(50);
        digitalWrite(22, HIGH);
        delay(50);
      }
      EEPROM.write(0, id);
      EEPROM.commit();

      Serial.print("NEW_ID: ");
      Serial.println(id);
      delay(200);
    }

    if( (millis()-last_packet) > 50 || id!=myData.ch16-127){
      digitalWrite(INA_1, LOW);
      digitalWrite(INA_2, LOW);
      //ledcWrite(ledChannel, 0);
      digitalWrite(INB_1, LOW);
      digitalWrite(INB_2, LOW);
      //ledcWrite(ledChanne2, 0);
      
      // //ledcWrite(ledChanne3, 0);
      // myservo.write(0); 
      Serial.print("NO SIGNAL my ch: ");
      Serial.print(id);
      Serial.print(" received id: ");
      Serial.println(myData.ch16-127);
    }else{
        if(myData.ch01>128){
        digitalWrite(INA_1, HIGH);
        digitalWrite(INA_2, LOW);
        ledcWrite(ledChanne4, ((myData.ch01-128)*2)+1);
        Serial.print("A for ward: ");
        Serial.println(((myData.ch01-128)*2)+1);
      }else if(myData.ch01==128){
        digitalWrite(INA_1, LOW);
        digitalWrite(INA_2, LOW);
        ledcWrite(ledChanne4, 0);
        Serial.println("A STOP");
      }else if(myData.ch01<128){
        digitalWrite(INA_1, LOW);
        digitalWrite(INA_2, HIGH);
        ledcWrite(ledChanne4, ((128-myData.ch01)*2)-1);
        Serial.print("A backward: ");
        Serial.println(((128-myData.ch01)*2)-1);
      }

      if(myData.ch02>128){
        digitalWrite(INB_1, HIGH);
        digitalWrite(INB_2, LOW);
        ledcWrite(ledChanne2, ((myData.ch02-128)*2)+1);
        Serial.print("B for ward: ");
        Serial.println(((myData.ch02-128)*2)+1);
      }else if(myData.ch02==128){
        digitalWrite(INB_1, LOW);
        digitalWrite(INB_2, LOW);
        ledcWrite(ledChanne2, 0);
        Serial.println("B STOP");
      }else if(myData.ch02<128){
        digitalWrite(INB_1, LOW);
        digitalWrite(INB_2, HIGH);
        ledcWrite(ledChanne2, ((128-myData.ch02)*2)-1);
        Serial.print("B backward: ");
        Serial.println(((128-myData.ch02)*2)-1);
      }
      
      // //ledcWrite(ledChanne3, myData.ch03);
      
      // myservo.write(map(myData.ch03,0,255,0,180));
      Serial.print("SERVO: ");
      Serial.println(map(myData.ch03,0,255,0,180));
    }
    
    

    // //ledcWrite(ledChannel, myData.ch01);
    // //ledcWrite(ledChanne2, myData.ch02);
  //   delay(15);
  // }

  // decrease the LED brightness
  // for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
  //   // changing the LED brightness with PWM
  //   //ledcWrite(ledChannel, dutyCycle);   
  //   //ledcWrite(ledChanne2, dutyCycle);   
  //   delay(15);
  // }


}
