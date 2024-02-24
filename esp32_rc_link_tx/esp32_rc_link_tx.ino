// esp_now
#include <esp_now.h>
#include <WiFi.h>
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned long last_sendtime = 0;

//analog pins
#define CH1 34
#define CH2 35
#define CH3 32
#define CH4 33
int ch1_offset = 0;
int ch2_offset = 0;

// motor values
int motorA = 0;
int motorB = 0;
uint8_t expo_B = 0;



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




#include <SoftwareSerial.h>

#define RX_PIN 5  // D5 on NodeMCU
#define TX_PIN 12  // D6 on NodeMCU 
#define SOFTWARE_BAUD 100000
#define MAX_BYTES 25

SoftwareSerial mySerial(RX_PIN, TX_PIN);

// parser
uint8_t buffer[25];
int bufferIndex = 0;
enum State {
  IDLE = 0,
  HEADER,
  HEADER2,
  HEADER3,
};
bool runonce = true;
uint16_t values[16];
uint8_t state = IDLE;
uint8_t b = 0;
uint32_t found_sequence = 0;
int indexx = 0;

// void extractValues(const uint8_t* buffer, uint16_t* outValues, int numValues = 16) {
//   int bitIndex = 0; // Index to track the current bit position in the buffer

//   yield();
//   for (int i = 0; i < numValues; ++i) {
//     yield();
//       // Extract 11 bits for each value
//       uint16_t value = 0;
//       for (int bit = 0; bit < 11; ++bit) {
//         yield();
//           // Calculate the index of the byte and the bit within that byte
//           int byteIndex = (bitIndex + bit) / 8;
//           int bitInByte = (bitIndex + bit) % 8;

//           // Extract the bit and add it to value
//           if (buffer[byteIndex] & (1 << bitInByte)) {
//               value |= (1 << bit);
              
//             yield();
//           }
//       }

//       // Store the extracted value
//       yield();
//       outValues[i] = map(value,173,1811,0,255);

//       // Move to the next 11-bit block
//       bitIndex += 11;
//   }
// }


// void report(){
//   if(buffer[22]==0x00){
    
//     yield();
//     extractValues(buffer, values, 16);
//     yield();
//     for (int i = 0; i < 16; ++i) {
//       Serial.print(values[i]);
//       Serial.print("\t");
      
//       yield();
//     }
    
//     myData.mode = 1;
//     myData.id   = 1;
//     myData.ch01 = values[0];
//     myData.ch02 = values[1];
//     myData.ch03 = values[2];
//     myData.ch04 = values[3];
//     myData.ch05 = values[4];
//     myData.ch06 = values[5];
//     myData.ch07 = values[6];
//     myData.ch08 = values[7];
//     myData.ch09 = values[8];
//     myData.ch10 = values[9];
//     myData.ch11 = values[10];
//     myData.ch12 = values[11];
//     myData.ch13 = values[12];
//     myData.ch14 = values[13];
//     myData.ch15 = values[14];
//     myData.ch16 = values[15];
//     yield();
    
//     if(millis()-last_sendtime >= 12){      
//       last_sendtime = millis();
//       esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
//     }
//     yield();

//     Serial.println();
//     // printBufferAsHex(buffer, 23);
//   }
//   else{
//     memset(buffer, 0, sizeof(buffer));
//     // clearRXBuffer();
//   }

//   // Serial.flush();
//   // switch_to_RX();
// }

// void parse_sbus(){
  
//   yield();
//   if (mySerial.available() > 0) {
//     switch(state){
      
//       yield();
//       case IDLE:
//         while(mySerial.available()>0){
//           yield();
//           b = mySerial.read();
//           if(b==0x00){
//             state=HEADER2;
//           }
//           else{
//             state=IDLE;
//           }
//         }
//         break;
//       case HEADER2:
//         if(mySerial.available()>23){
//           b = mySerial.read();
//           if(b==0x0F){
//             memset(buffer, 0, sizeof(buffer));
//             mySerial.readBytes(buffer, 23);
//             state=IDLE;
//             found_sequence++;
            
//             // serial_count_values();
//             // report_periodically();
//             report();
            
//             yield();
//             // report_mapped();
//             // report_mapped_assembled();

//           }else if(b==0x00){
//             break;
//           }else{
//             state=IDLE;
//           }
//         }
//         break;
//     }
//   }
// }

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
    ch1_offset += analogRead(CH1);
    ch2_offset += analogRead(CH2);
    delay(1);
  }
  ch1_offset = ch1_offset/500;
  ch2_offset = ch2_offset/500;
  
  Serial.println("Calibration done");
}

uint8_t calculate_expo(int rc_in, float expo){

  float scaled_rc = ((float)rc_in/2047)-1;
  float expo_factor = scaled_rc * (1 - expo + (expo * scaled_rc * scaled_rc));

  uint8_t expo_val = (uint8_t)((expo_factor+1)*127.5);


  // Serial.print("\tin: ");
  // Serial.print(scaled_rc);
  // Serial.print("\tout: ");
  // Serial.print(expo_factor);
  // Serial.print("\tExpo: ");
  // Serial.println(expo_val);

  return expo_val;

}

void read_analog(){
  
  yield();

  motorA = mapWithMidpoint(analogRead(CH1), 4095, ch1_offset, 0, 0, 255);
  motorB = mapWithMidpoint(analogRead(CH1), 4095, ch1_offset, 0, 0, 255);
  
  expo_B = calculate_expo(mapWithMidpoint(analogRead(CH2), 0, ch2_offset, 4095, 0, 4095), 0.7);
  // Serial.println(mapWithMidpoint(analogRead(CH2), 0, ch2_offset, 4095, 0, 4095));

  motorA += (expo_B-128)/4;
  motorB -= (expo_B-128)/4;

  motorA = constrain(motorA, 0, 255);
  motorB = constrain(motorB, 0, 255);

  myData.mode = 1;
  myData.id   = 1;
  myData.ch01 = motorA;
  myData.ch02 = motorB;
  myData.ch03 = mapWithMidpoint(analogRead(CH3),0,1799,4095,0,255);
  myData.ch04 = mapWithMidpoint(analogRead(CH4),0,1799,4095,0,255);
  myData.ch05 = values[4];
  myData.ch06 = values[5];
  myData.ch07 = values[6];
  myData.ch08 = values[7];
  myData.ch09 = values[8];
  myData.ch10 = values[9];
  myData.ch11 = values[10];
  myData.ch12 = values[11];
  myData.ch13 = values[12];
  myData.ch14 = values[13];
  myData.ch15 = values[14];
  myData.ch16 = 130;
  yield();
  
  
  Serial.print("stearing: ");
  Serial.print(motorA);
  Serial.print("A: ");
  Serial.print(motorA);
  Serial.print(" B: ");
  Serial.println(motorB);
  
  if(millis()-last_sendtime >= 12){      
    last_sendtime = millis();
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
}

void setup() {

  
    pinMode(CH1, INPUT);
    pinMode(CH2, INPUT);
    pinMode(CH3, INPUT);
    pinMode(CH4, INPUT);
    delay(500);
    calibrate();
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

  Serial.begin(115200);
  


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

//   mySerial.begin(SOFTWARE_BAUD, SWSERIAL_8E2, RX_PIN, TX_PIN, true);  // true for inverted signal
}

void loop() {
  
  yield();

//   parse_sbus();
    read_analog();

  yield();
}

