// general
enum TX_STATES {
  IDLE,
  SENDING,
  BINDING,
};
uint8_t state = SENDING;
bool new_rx_data = false;

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
#include "esp_wifi.h"
#include <esp_now.h>
#include <WiFi.h>
#define rssi_list_size 50
#define binding_ch 14
uint8_t current_ch = 0;
uint8_t sending_ch = 5;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct rssi_data {
  uint8_t mac_address[6];
  int rssi;
  uint8_t received_packets;
};
rssi_data rssi_list[rssi_list_size] = {0};
rssi_data rssi_receive_buffer[rssi_list_size] = {0};
uint8_t rssi_list_index = 0;
uint8_t rssi_receive_buffer_index = 0;
uint8_t strongest_rssi_index = 0;

unsigned long last_strongest_receive = 0;
unsigned long last_sendtime = 0;
unsigned long last_receive=0;
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
  char string[15];
} struct_message;
struct_message myData;
int ch1_offset = 0;
int ch2_offset = 0;
int motorA = 0;
int motorB = 0;
uint8_t expo_B = 0;

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

void print_rssi_list(){
  for (int i = 0; i < rssi_list_index; i++) {
    Serial.print("=======================");
    Serial.print("INDEX: ");
    Serial.println(i);
    Serial.print("MAC: ");
    print_MAC(rssi_list[i].mac_address);
    Serial.print("RSSI: ");
    Serial.println(rssi_list[i].rssi);
    Serial.print("RECEIVED PACKETS: ");
    Serial.println(rssi_list[i].received_packets);
  }
}

unsigned long last_list_print = 0;
void binding(){
  if(current_ch != binding_ch)change_channel(binding_ch);
  int strongest_rssi_index_local = 0;
  int strongest_rssi_local = -200;
  for (int i = 0; i < rssi_list_index; i++) {
    if(rssi_list[i].rssi > -50 && rssi_list[i].rssi > strongest_rssi_local){
      strongest_rssi_local = i;
    }
  }  
  if(strongest_rssi_local != -200 && strongest_rssi_local != strongest_rssi_index){
    strongest_rssi_index = strongest_rssi_local;
    last_strongest_receive = millis();
  }

  if(millis()-last_strongest_receive > 1000 && rssi_list[strongest_rssi_index].received_packets > 10 && rssi_list[strongest_rssi_index].rssi > -50 ){
    Serial.println("BINDING DONE");
    Serial.print("STRONGEST RSSI INDEX: ");
    Serial.println(strongest_rssi_index);
    Serial.print("STRONGEST RSSI VALUE: ");
    Serial.println(rssi_list[strongest_rssi_index].rssi);
    Serial.print("MAC: ");
    print_MAC(rssi_list[strongest_rssi_index].mac_address);    
    Serial.print("  ");    
    print_rssi_list();

    myData.mode   = 43;
    myData.id     = 43;
    myData.x_axis = 43;
    myData.y_axis = 43;
    myData.pot_1  = 43;
    myData.sw_1   = 43;
    myData.sw_2   = 43;
    myData.ch10   = 2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    delay(100);
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    delay(100);
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    delay(100);
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    sending_ch = 2;
    change_channel(sending_ch);
    Serial.println("binding confirmed");
    memcpy(peerInfo.peer_addr, rssi_list[strongest_rssi_index].mac_address, 6);
    peerInfo.channel = sending_ch;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    Serial.print("Added: ");
    print_MAC(peerInfo.peer_addr);
    Serial.print("Channel: ");
    Serial.println(peerInfo.channel);
  
    state = SENDING;
    return;

  }else if(millis()-last_list_print > 1000){
    last_list_print = millis();
    Serial.print("Time since last strongest receive: ");
    Serial.println(millis() - last_strongest_receive);
    print_rssi_list();
  }

  if(millis()-last_sendtime > 200){
    last_sendtime = millis();
    myData.mode   = 42;
    myData.id     = 42;
    myData.x_axis = 42;
    myData.y_axis = 42;
    myData.pot_1  = 42;
    myData.sw_1   = 42;
    myData.sw_2   = 42;
    myData.ch06   = 42;
    myData.ch07   = 42;
    myData.ch08   = 42;
    myData.ch09   = 42;
    myData.ch10   = 42;
    myData.ch11   = 42;
    myData.ch12   = 42;
    myData.ch13   = 42;
    myData.ch14   = 42;
    myData.ch15   = 42;
    myData.ch16   = 42;
    // Serial.println("SENDING BINDING");
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
}

void change_channel(uint8_t channel){
  esp_wifi_set_channel(channel,WIFI_SECOND_CHAN_NONE);
  current_ch = channel;
}

void print_MAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
}

bool received_binding_packet(){
  if(myData.mode == 42
  && myData.id == 42
  && myData.x_axis == 42
  && myData.y_axis == 42
  && myData.pot_1 == 42
  && myData.sw_1 == 42
  && myData.sw_2 == 42){
    return true;
  } else {
    return false;
  }  
}

int findMacAddress(const uint8_t* mac_address) {
  for (int i = 0; i < rssi_list_index; i++) {
    if (memcmp(rssi_list[i].mac_address, mac_address, sizeof(rssi_list[i].mac_address)) == 0) {
      return i;
    }
  }
  return -1;
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  last_receive = millis();
  
  if(state == BINDING){
    if(received_binding_packet()){
      int index = findMacAddress(mac);
      
      if(index == -1){
        if(rssi_list_index < rssi_list_size){
          for (int i = 0; i < 6; i++) {
            rssi_list[rssi_list_index].mac_address[i] = mac[i];
          }
          // memcpy(rssi_list[rssi_list_index].mac_address, mac, sizeof(mac));
          rssi_list[rssi_list_index].rssi = rssi_last;
          rssi_list[rssi_list_index].received_packets = 1;
          rssi_list_index++;
        }else{
          Serial.println("RSSI LIST FULL");
          while (true)
          {
            delay(1000);
          }
          
        }
      }else{
        for (int i = 0; i < 6; i++) {
          rssi_list[index].mac_address[i] = mac[i];
        }
        rssi_list[index].rssi = rssi_last;
        rssi_list[index].received_packets++;
      }
      // Serial.print("Binding packet received from: ");
      // print_MAC(mac);
      // Serial.print("RSSI: ");
      // Serial.println(rssi_last);
      // Serial.print("Index: ");
      // Serial.println(index);
      // Serial.print("Mac for index: ");
      // print_MAC(rssi_list[index].mac_address);
      // Serial.print("Received packets: ");
      // Serial.println(rssi_list[index].received_packets);
    }
  }else if(state == SENDING){
    new_rx_data = true;
  }

  // Serial.print("RECIEVED: ");
  // Serial.print(myData.x_axis);
  // Serial.print("\t");
  // Serial.print(myData.y_axis);
  // Serial.print("\t");
  // Serial.print(myData.pot_1);
  // Serial.print("\t");
  // Serial.print(myData.sw_1);
  // Serial.print("\t");
  // Serial.println(myData.sw_2);
  
}

void init_esp_now(){
  WiFi.mode(WIFI_STA);
  
  esp_wifi_set_channel(sending_ch,WIFI_SECOND_CHAN_NONE);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = binding_ch;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  if(state == BINDING){
    esp_now_register_recv_cb(OnDataRecv);
    change_channel(binding_ch);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
  }else if(state == SENDING){
    change_channel(sending_ch);    
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);    
    esp_now_register_recv_cb(OnDataRecv);    
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

uint32_t calculate_expo_12_Bit(int rc_in, float expo){

  float scaled_rc = ((float)rc_in/2047)-1;
  scaled_rc = constrain(scaled_rc, -1.0, 1.0);
  float expo_factor = scaled_rc * (1 - expo + (expo * scaled_rc * scaled_rc));
  uint32_t expo_val = (uint32_t)((expo_factor+1)*2048.0);
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

  calculate_expo_12_Bit(mapWithMidpoint(constrain(analogRead(g_y),EEPROM_DATA.calib_y_low,EEPROM_DATA.calib_y_high), EEPROM_DATA.calib_y_low, ch2_offset, EEPROM_DATA.calib_y_high, 0, 4095), 0.3);

  myData.mode   = 1;
  myData.id     = 1;
  if(get_switch_pos_2()==1){
    myData.x_axis = calculate_expo_12_Bit(mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095),0.5);
  }else if(get_switch_pos_2()==2){
    myData.x_axis = calculate_expo_12_Bit(mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095),0.25);
  }else if(get_switch_pos_2()==3){
    myData.x_axis = mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095);
  }
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

  // esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  esp_now_send(rssi_list[strongest_rssi_index].mac_address, (uint8_t *) &myData, sizeof(myData));
}

void update_states(){
  if(get_switch_pos_1()==3 && analogRead(pot) < 50){
    state = BINDING;
  }
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
  state = BINDING;

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
  // update_states();
  switch (state) {
    case SENDING:
      if(millis()-last_sendtime > 50){
        last_sendtime = millis();
        send_joysitck();
      }
      break;

    case BINDING:      
      binding();
      break;
  }

  
  // Serial.print(analogRead(g_x));
  // Serial.print("\t");
  // Serial.print(analogRead(g_y));
  // Serial.print("\t");
  // Serial.println(analogRead(pot));
}
