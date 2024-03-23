// eeprom code start
#define EEPROM_SIZE 200
#define EEPROM_ADDRES 100
#include <EEPROM.h>

typedef struct struct_eeprom {
  uint8_t   eeprom_structure_version;
  uint8_t   reserved;
  uint8_t   need_to_calibrate;
  int32_t   calib_x_low;
  int32_t   calib_x_high;
  int32_t   calib_y_low;
  int32_t   calib_y_high;
  int32_t   offset_x;
  int32_t   offset_y;
} struct_eeprom;
struct_eeprom EEPROM_DATA;

void init_eeprom(){  
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRES, EEPROM_DATA);
}
// eeprom code end

// esp now
#include <esp_now.h>
#include <WiFi.h>
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned long last_sendtime = 0;
esp_now_peer_info_t peerInfo;

// hardware pins
#define g_x 9
#define g_y 10
#define pot 8
#define switch_1_up   15
#define switch_1_down 16
#define switch_2_up   17
#define switch_2_down 18

// variables
typedef struct struct_message {
  uint8_t   mode;
  uint8_t   id;
  int32_t   x_axis;
  int32_t   y_axis;
  uint32_t  pot_1;
  uint8_t   sw_1;
  uint8_t   sw_2;
  uint8_t   ch06;
  uint8_t   ch07;
  uint8_t   ch08;
  uint8_t   ch09;
  uint8_t   ch10;
  uint8_t   ch11;
  uint8_t   ch12;
  uint8_t   ch13;
  uint8_t   ch14;
  uint8_t   ch15;
  uint8_t   ch16;
} struct_message;
struct_message myData;
int ch1_offset = 0;
int ch2_offset = 0;
int motorA = 0;
int motorB = 0;
uint8_t expo_B = 0;

void init_data_structures(){
  myData.mode   = 0;
  myData.id     = 0;
  myData.x_axis = 0;
  myData.y_axis = 0;
  myData.pot_1  = 0;
  myData.sw_1   = 0;
  myData.sw_2   = 0;
  myData.ch06   = 0;
  myData.ch07   = 0;
  myData.ch08   = 0;
  myData.ch09   = 0;
  myData.ch10   = 0;
  myData.ch11   = 0;
  myData.ch12   = 0;
  myData.ch13   = 0;
  myData.ch14   = 0;
  myData.ch15   = 0;
  myData.ch16   = 0;
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


void calibrate_joystick(){
  Serial.println("Calibrating josystick midpoint...");
  ch1_offset = 0;
  ch2_offset = 0;
  for(int i=0; i<5000; i++){
    ch1_offset += analogRead(g_x);
    ch2_offset += analogRead(g_y);
    delay(1);
  }
  ch1_offset = ch1_offset/5000;
  ch2_offset = ch2_offset/5000;
  
  Serial.println("midpoint calibration done\n");
  EEPROM_DATA.need_to_calibrate = 0;
  EEPROM_DATA.offset_x = ch1_offset;
  EEPROM_DATA.offset_y = ch2_offset;

  // calibrate joystick limits
  Serial.println("Calibrating joystick limits...");
  Serial.println("move joystick to all limits for 10sec");
  delay(1000);
  unsigned long start_time = millis();
  uint16_t x_low  = 2048;
  uint16_t x_high = 2048;
  uint16_t y_low  = 2048;
  uint16_t y_high = 2048;
  while (millis()-start_time < 10000)
  {
    x_low = min(x_low, analogRead(g_x));
    x_high = max(x_high, analogRead(g_x));
    y_low = min(y_low, analogRead(g_y));
    y_high = max(y_high, analogRead(g_y));

    delay(10);
  }
  
  Serial.println("joystick limits calibration done");
  Serial.print("calib_x_low = ");
  Serial.println(x_low);
  Serial.print("calib_x_high = ");
  Serial.println(x_high);
  Serial.print("calib_y_low = ");
  Serial.println(y_low);
  Serial.print("calib_y_high = ");
  Serial.println(y_high);

  EEPROM_DATA.calib_x_low  = x_low;
  EEPROM_DATA.calib_x_high = x_high;
  EEPROM_DATA.calib_y_low  = y_low;
  EEPROM_DATA.calib_y_high = y_high;

  EEPROM.put(EEPROM_ADDRES, EEPROM_DATA);
  EEPROM.commit();
}

uint8_t calculate_expo(int rc_in, float expo){

  float scaled_rc = ((float)rc_in/2047)-1;
  scaled_rc = constrain(scaled_rc, -1.0, 1.0);
  float expo_factor = scaled_rc * (1 - expo + (expo * scaled_rc * scaled_rc));
  uint8_t expo_val = (uint8_t)((expo_factor+1)*127.5);
  return expo_val;

}

uint8_t get_switch_pos_1(){
  bool sw_pos_up = digitalRead(switch_1_up);
  bool sw_pos_down = digitalRead(switch_1_down);
  if(sw_pos_up && sw_pos_down){
    return 2;
  }
  if(!sw_pos_up && sw_pos_down){
    return 1;
  }
  if(sw_pos_up && !sw_pos_down){
    return 3;
  }
}

uint8_t get_switch_pos_2(){
  bool sw_pos_up = digitalRead(switch_2_up);
  bool sw_pos_down = digitalRead(switch_2_down);
  if(sw_pos_up && sw_pos_down){
    return 2;
  }
  if(!sw_pos_up && sw_pos_down){
    return 1;
  }
  if(sw_pos_up && !sw_pos_down){
    return 3;
  }
}

void send_joysitck(){

  myData.mode   = 1;
  myData.id     = 1;
  myData.x_axis = mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095);
  myData.y_axis = 4095 - mapWithMidpoint(constrain(analogRead(g_y),EEPROM_DATA.calib_y_low ,EEPROM_DATA.calib_y_high), EEPROM_DATA.calib_y_low, ch2_offset, EEPROM_DATA.calib_y_high, 0, 4095);
  myData.pot_1  = analogRead(pot);
  myData.sw_1   = get_switch_pos_1();
  myData.sw_2   = get_switch_pos_2();
  myData.ch06   = 50;
  myData.ch07   = 50;
  myData.ch08   = 50;
  myData.ch09   = 50;
  myData.ch10   = 50;
  myData.ch11   = 50;
  myData.ch12   = 50;
  myData.ch13   = 50;
  myData.ch14   = 50;
  myData.ch15   = 50;
  myData.ch16   = 130;

  // Serial.print(myData.x_axis);
  // Serial.print("\t");
  // Serial.print(myData.y_axis);
  // Serial.print("\t");
  // Serial.print(myData.pot_1);
  // Serial.print("\t");
  // Serial.print(myData.sw_1);
  // Serial.print("\t");
  // Serial.println(myData.sw_2);

  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
}

void init_gpio(){
  pinMode(g_x, INPUT);
  pinMode(g_y, INPUT);
  pinMode(pot, INPUT);
  pinMode(switch_1_up  , INPUT_PULLUP);
  pinMode(switch_1_down, INPUT_PULLUP);
  pinMode(switch_2_up  , INPUT_PULLUP);
  pinMode(switch_2_down, INPUT_PULLUP);
  delay(150);
}

void setup() {
  Serial.begin(115200);
  init_eeprom();
  init_gpio();
  init_data_structures();
  init_esp_now();
  if(EEPROM_DATA.need_to_calibrate){
    calibrate_joystick();
  }else{
    ch1_offset = EEPROM_DATA.offset_x;
    ch2_offset = EEPROM_DATA.offset_y;
  }

  Serial.print(" eeprom_structure_version = ");
  Serial.println(EEPROM_DATA.eeprom_structure_version);
  Serial.print(" reserved = ");
  Serial.println(EEPROM_DATA.reserved);
  Serial.print(" calibration needed = ");
  Serial.println(EEPROM_DATA.need_to_calibrate);
  Serial.print(" calib_x_low = ");
  Serial.println(EEPROM_DATA.calib_x_low);
  Serial.print(" calib_x_high = ");
  Serial.println(EEPROM_DATA.calib_x_high);
  Serial.print(" calib_y_low = ");
  Serial.println(EEPROM_DATA.calib_y_low);
  Serial.print(" calib_y_high = ");
  Serial.println(EEPROM_DATA.calib_y_high);
  Serial.print(" offset_x = ");
  Serial.println(EEPROM_DATA.offset_x);
  Serial.print(" offset_y = ");
  Serial.println(EEPROM_DATA.offset_y);
  
}

void loop() {
  send_joysitck();
  // Serial.print(analogRead(pot));
  // Serial.print("\t");
  // Serial.print(digitalRead(switch_1_up));
  // Serial.print("\t");
  // Serial.print(digitalRead(switch_1_down));
  // Serial.print("\t");
  // Serial.print(digitalRead(switch_2_up));
  // Serial.print("\t");
  // Serial.println(digitalRead(switch_2_down));
  delay(10);
}
