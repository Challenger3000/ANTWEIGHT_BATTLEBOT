// esp_now
#include <ESP8266WiFi.h>
#include <espnow.h>
// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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


void extractValues(const uint8_t* buffer, uint16_t* outValues, int numValues = 16) {
  int bitIndex = 0; // Index to track the current bit position in the buffer

  yield();
  for (int i = 0; i < numValues; ++i) {
    yield();
      // Extract 11 bits for each value
      uint16_t value = 0;
      for (int bit = 0; bit < 11; ++bit) {
        yield();
          // Calculate the index of the byte and the bit within that byte
          int byteIndex = (bitIndex + bit) / 8;
          int bitInByte = (bitIndex + bit) % 8;

          // Extract the bit and add it to value
          if (buffer[byteIndex] & (1 << bitInByte)) {
              value |= (1 << bit);
              
            yield();
          }
      }

      // Store the extracted value
      yield();
      outValues[i] = map(value,173,1811,0,255);

      // Move to the next 11-bit block
      bitIndex += 11;
  }
}

unsigned long last_sendtime = 0;

void serial_report(){
  if(buffer[22]==0x00){
    
    yield();
    extractValues(buffer, values, 16);
    yield();
    for (int i = 0; i < 16; ++i) {
      Serial.print(values[i]);
      Serial.print("\t");
      
      yield();
    }
    
    myData.mode = 1;
    myData.id   = 1;
    myData.ch01 = values[0];
    myData.ch02 = values[1];
    myData.ch03 = values[2];
    myData.ch04 = values[3];
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
    myData.ch16 = values[15];
    yield();
    
    if(millis()-last_sendtime >= 12){      
      last_sendtime = millis();
      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    }
    yield();

    Serial.println();
    // printBufferAsHex(buffer, 23);
  }
  else{
    memset(buffer, 0, sizeof(buffer));
    // clearRXBuffer();
  }

  // Serial.flush();
  // switch_to_RX();
}

void parse_sbus(){
  
  yield();
  if (mySerial.available() > 0) {
    switch(state){
      
      yield();
      case IDLE:
        while(mySerial.available()>0){
          yield();
          b = mySerial.read();
          if(b==0x00){
            state=HEADER2;
          }
          else{
            state=IDLE;
          }
        }
        break;
      case HEADER2:
        if(mySerial.available()>23){
          b = mySerial.read();
          if(b==0x0F){
            memset(buffer, 0, sizeof(buffer));
            mySerial.readBytes(buffer, 23);
            state=IDLE;
            found_sequence++;
            
            // serial_count_values();
            // serial_report_periodically();
            serial_report();
            
            yield();
            // serial_report_mapped();
            // serial_report_mapped_assembled();

          }else if(b==0x00){
            break;
          }else{
            state=IDLE;
          }
        }
        break;
    }
  }
}

void setup() {
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
  
    // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);


  // Initialize software serial with inversion
  mySerial.begin(SOFTWARE_BAUD, SWSERIAL_8E2, RX_PIN, TX_PIN, true);  // true for inverted signal
}

void loop() {
  
  yield();

  parse_sbus();
  
  yield();
}

