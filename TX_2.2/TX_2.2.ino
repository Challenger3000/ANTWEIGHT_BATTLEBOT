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
  uint8_t   binding_status;
  uint8_t   bound_ch;
  uint8_t   bound_mac[6];
  char      encryption_key[16];
} struct_eeprom;
struct_eeprom EEPROM_DATA;

void init_eeprom(){  
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRES, EEPROM_DATA);
	// Serial.println();
	// Serial.print("EEPROM Binding Status: ");
	// Serial.println(EEPROM_DATA.binding_status);
	// Serial.print("EEPROM Bound Channel: ");
	// Serial.println(EEPROM_DATA.bound_ch);
	// Serial.print("EEPROM Bound MAC Address: ");
	// for (int i = 0; i < 6; i++) {
	// 		Serial.print(EEPROM_DATA.bound_mac[i], HEX);
	// 		if (i < 5) {
	// 				Serial.print(":");
	// 		}
	// }
	// Serial.println();
	// Serial.print("EEPROM Encryption Key Size: ");
	// Serial.println(sizeof(EEPROM_DATA.encryption_key));
	// Serial.print("EEPROM Encryption Key: ");
	// for (int i = 0; i < 16; i++) {
	// 	Serial.print(EEPROM_DATA.encryption_key[i], HEX);
	// 	if (i < 15) {
	// 		Serial.print(":");
	// 	}
	// }
	// Serial.println();
}
// eeprom code end

// esp now code start
#include "esp_wifi.h"
#include <esp_now.h>
#include <WiFi.h>
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
#define rssi_list_size 50
#define binding_ch 14
uint8_t current_ch = 0;
uint8_t sending_ch = 5;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
char pmk_key_str[16];
uint16_t failed_packet_count = 0;
bool rx_lost = true;

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
// esp now code end

// hardware pins
#define g_x 4
#define g_y 5
#define pot 6
#define switch_1      14
#define switch_2_up   12
#define switch_2_down 11
#define switch_3      13
#define BTN_A 9
#define BTN_B 21
#define vbat 8

// variables
typedef struct struct_message_rx {
  float     volatage;
} struct_message_rx;
struct_message_rx rxData;

typedef struct struct_message_tx {
  uint8_t   mode;
  uint8_t   id;
  int32_t   x_axis;
  int32_t   y_axis;
  uint32_t  pot_1;
  uint8_t   sw_1;
  uint8_t   sw_2;
  uint8_t   sw_3;
  uint8_t   btn_A;
  uint8_t   btn_B;
  uint8_t   ch09;
  uint8_t   ch10;
  uint8_t   ch11;
  uint8_t   ch12;
  uint8_t   ch13;
  uint8_t   ch14;
  uint8_t   ch15;
  uint8_t   ch16;
  char string[16];
  uint8_t mac[6];
} struct_message_tx;
struct_message_tx txData;
int ch1_offset = 0;
int ch2_offset = 0;
int motorA = 0;
int motorB = 0;
uint8_t expo_B = 0;


// led code start
#include <FastLED.h>
#define NUM_LEDS 4
#define DATA_PIN 1
CRGB leds[NUM_LEDS];
unsigned long last_led_update = 0;
bool led_warning_phase = false;

void init_led(){
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB
  leds[0] = CRGB(10, 10, 10);
  FastLED.show();
  delay(10);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
}

void led_color(uint8_t led_number,uint8_t red, uint8_t green, uint8_t blue){
  if(led_number < NUM_LEDS && led_number >= 0){
    leds[led_number] = CRGB(green, red, blue);
    FastLED.show();
  }
  FastLED.show();
}
void update_leds(){
  // led 0 is showing battery status  
  // Serial.println();
  // Serial.println((float)analogRead(vbat)/600.0);
  float battery_voltage = (float)analogRead(vbat)/600.0;
  if(battery_voltage > 4.0){
    led_color(0, 0, 255, 0);
  }else if(battery_voltage > 3.8){
    led_color(0, 255, 255, 0);
  }else if(battery_voltage > 3.5){
    led_color(0, 255, 0, 0);
  }else if(battery_voltage > 3.1){
    if(millis() % 500 < 250){
      led_color(0, 0, 0, 0);
    }else{
      led_color(0, 255, 0, 0);
    }
  }

  // led 1 is showing robots battery status
  // 4.1v - green
  // 3.8v - yellow
  // 3.5v - red
  // 3.1v - red blinking
  if(state == BINDING){
    if(millis() % 500 < 250){
      led_color(1, 0, 0, 255);
    }else{
      led_color(1, 0, 70, 128);
    }
  }else if(rx_lost){
    led_color(1, 0, 0, 0);
  }else if(rxData.volatage > 4.0){
    led_color(1, 0, 255, 0);
  }else if(rxData.volatage > 3.8){
    led_color(1, 255, 255, 0);
  }else if(rxData.volatage > 3.5){
    led_color(1, 255, 0, 0);
  }else if(rxData.volatage > 3.1){
    if(millis() % 500 < 250){
      led_color(1, 0, 0, 0);
    }else{
      led_color(1, 255, 0, 0);
    }
  }

  // led 2 is showing connection status  
  led_color(2, 10, 10, 10);

  // led 3 is free, to be used as a general purpouse indicator
  led_color(3, 10, 10, 10);
}
// led code end

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
  txData.mode   = 0;
  txData.id     = 0;
  txData.x_axis = 0;
  txData.y_axis = 0;
  txData.pot_1  = 0;
  txData.sw_1   = 0;
  txData.sw_2   = 0;
  txData.sw_3   = 0;
  txData.btn_A   = 0;
  txData.btn_B   = 0;
  txData.ch09   = 0;
  txData.ch10   = 0;
  txData.ch11   = 0;
  txData.ch12   = 0;
  txData.ch13   = 0;
  txData.ch14   = 0;
  txData.ch15   = 0;
  txData.ch16   = 0;
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
    // Serial.println("BINDING DONE");
    // Serial.print("STRONGEST RSSI INDEX: ");
    // Serial.println(strongest_rssi_index);
    // Serial.print("STRONGEST RSSI VALUE: ");
    // Serial.println(rssi_list[strongest_rssi_index].rssi);
    // Serial.print("MAC: ");
    // print_MAC(rssi_list[strongest_rssi_index].mac_address);    
    // Serial.print("  ");    
    // print_rssi_list();

    randomSeed((unsigned long)esp_random());
    pmk_key_str[0] = '\0';  
    // Serial.println("Generating a random 16-byte PMK:");

    // Append each random byte as a two-character hex value to the string
    for (int i = 0; i < 16; i++) {
      uint8_t randomByte = random(0, 256);  // Generate a random byte
      pmk_key_str[i] = randomByte;  // Format byte as hex and append
    }

    sending_ch = random(0, 13);
    txData.mode   = 43;
    txData.id     = 43;
    txData.x_axis = 43;
    txData.y_axis = 43;
    txData.pot_1  = 43;
    txData.sw_1   = 43;
    txData.sw_2   = 43;
    txData.ch10   = sending_ch;
    strcpy(txData.string, pmk_key_str);
    memcpy(txData.mac, rssi_list[strongest_rssi_index].mac_address, 6);


    // Serial.print("Password: ");
    // for (int i = 0; i < 16; i++) {
    //   Serial.print("0x");
    //   Serial.print((byte)txData.string[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();
    
    EEPROM_DATA.binding_status = 1;
    EEPROM_DATA.bound_ch = sending_ch;
    memcpy(EEPROM_DATA.bound_mac, rssi_list[strongest_rssi_index].mac_address, sizeof(EEPROM_DATA.bound_mac));
    memcpy(EEPROM_DATA.encryption_key, txData.string, sizeof(txData.string));
    EEPROM.put(EEPROM_ADDRES, EEPROM_DATA);
    EEPROM.commit();

    esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
    delay(100);
    esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
    delay(100);
    esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
    delay(100);
    esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
    esp_wifi_set_promiscuous(false);

    change_channel(sending_ch);
    // Serial.println("binding confirmed");
    memcpy(peerInfo.peer_addr, rssi_list[strongest_rssi_index].mac_address, 6);
    peerInfo.channel = sending_ch;
    peerInfo.encrypt = true;
    memcpy(peerInfo.lmk, txData.string, 16);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    // Serial.print("Added: ");
    // print_MAC(peerInfo.peer_addr);
    // Serial.print("Channel: ");
    // Serial.println(peerInfo.channel);    
    esp_now_register_send_cb(OnDataSent);
  
    state = SENDING;
    return;

  }

  if(millis()-last_sendtime > 200){
    last_sendtime = millis();
    txData.mode   = 42;
    txData.id     = 42;
    txData.x_axis = 42;
    txData.y_axis = 42;
    txData.pot_1  = 42;
    txData.sw_1   = 42;
    txData.sw_2   = 42;
    txData.sw_3   = 42;
    txData.btn_A  = 42;
    txData.btn_B  = 42;
    txData.ch09   = 42;
    txData.ch10   = 42;
    txData.ch11   = 42;
    txData.ch12   = 42;
    txData.ch13   = 42;
    txData.ch14   = 42;
    txData.ch15   = 42;
    txData.ch16   = 42;
    // Serial.println("SENDING BINDING");
    esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
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
  if(txData.mode == 42
  && txData.id == 42
  && txData.x_axis == 42
  && txData.y_axis == 42
  && txData.pot_1 == 42
  && txData.sw_1 == 42
  && txData.sw_2 == 42){
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

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS){
    failed_packet_count = 0;
    rx_lost = false;
  }else{
    failed_packet_count++;
    if(failed_packet_count > 5){
      rx_lost = true;
    }
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&txData, incomingData, sizeof(txData));
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
    memcpy(&rxData, incomingData, sizeof(rxData));
    // Serial.print("Bytes received: ");
    // Serial.println(len);
    // Serial.print("v: ");
    // Serial.println(rxData.volatage);
    // Serial.println();

  }

  // Serial.print("RECIEVED: ");
  // Serial.print(txData.x_axis);
  // Serial.print("\t");
  // Serial.print(txData.y_axis);
  // Serial.print("\t");
  // Serial.print(txData.pot_1);
  // Serial.print("\t");
  // Serial.print(txData.sw_1);
  // Serial.print("\t");
  // Serial.println(txData.sw_2);
  
}

void init_esp_now(){
  WiFi.mode(WIFI_STA);
  
  esp_wifi_set_channel(sending_ch,WIFI_SECOND_CHAN_NONE);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  if(state == BINDING){
    Serial.println("Binding mode");
    // Register binding peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = binding_ch;  
    peerInfo.encrypt = false;    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    change_channel(binding_ch);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
  }else if(state == SENDING){
    sending_ch = EEPROM_DATA.bound_ch;
    memcpy(peerInfo.peer_addr, EEPROM_DATA.bound_mac, 6);
    peerInfo.channel = EEPROM_DATA.bound_ch;  
    peerInfo.encrypt = true;      
    memcpy(peerInfo.lmk, EEPROM_DATA.encryption_key, 16);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    // Serial.println("Sending mode");
    change_channel(sending_ch);
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
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
  // Serial.print("calib_x_low = ");
  // Serial.println(x_low);
  // Serial.print("calib_x_high = ");
  // Serial.println(x_high);
  // Serial.print("calib_y_low = ");
  // Serial.println(y_low);
  // Serial.print("calib_y_high = ");
  // Serial.println(y_high);

  EEPROM_DATA.calib_x_low  = x_low + 100;
  EEPROM_DATA.calib_x_high = x_high - 100;
  EEPROM_DATA.calib_y_low  = y_low + 100;
  EEPROM_DATA.calib_y_high = y_high - 100;

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

int get_pot(){
  return analogRead(pot);
}

uint8_t get_switch_1(){
  return !digitalRead(switch_1);
}

uint8_t get_switch_2(){
  bool sw_pos_up = digitalRead(switch_2_up);
  bool sw_pos_down = digitalRead(switch_2_down);
  if(sw_pos_up && sw_pos_down){
    return 1;
  }
  if(!sw_pos_up && sw_pos_down){
    return 0;
  }
  if(sw_pos_up && !sw_pos_down){
    return 2;
  }
}

uint8_t get_switch_3(){
  return digitalRead(switch_3);
}

uint8_t get_button_A(){
  return !digitalRead(BTN_A);
}

uint8_t get_button_B(){
  return !digitalRead(BTN_B);
}

void send_joysitck(){
  calculate_expo_12_Bit(mapWithMidpoint(constrain(analogRead(g_y),EEPROM_DATA.calib_y_low,EEPROM_DATA.calib_y_high), EEPROM_DATA.calib_y_low, ch2_offset, EEPROM_DATA.calib_y_high, 0, 4095), 0.3);

  txData.mode   = 1;
  txData.id     = 1;
  // if(get_switch_2()==1){
  if(false){
    txData.x_axis = 4095 -  calculate_expo_12_Bit(mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095),0.5);
  // }else if(get_switch_2()==2){
  }else if(true){
    txData.x_axis = 4095 -  calculate_expo_12_Bit(mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095),0.25);
  // }else if(get_switch_2()==3){
  }else if(false){
    txData.x_axis = 4095 - mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095);
  }
  txData.y_axis =mapWithMidpoint(constrain(analogRead(g_y),EEPROM_DATA.calib_y_low ,EEPROM_DATA.calib_y_high), EEPROM_DATA.calib_y_low, ch2_offset, EEPROM_DATA.calib_y_high, 0, 4095);
  txData.pot_1  = analogRead(pot);
  txData.sw_1   = get_switch_1();
  txData.sw_2   = get_switch_2();
  txData.sw_3   = get_switch_2();
  txData.btn_A  = get_button_A();
  txData.btn_B  = get_button_B();
  txData.ch09   = 50;
  txData.ch10   = 50;
  txData.ch11   = 50;
  txData.ch12   = 50;
  txData.ch13   = 50;
  txData.ch14   = 50;
  txData.ch15   = 50;
  txData.ch16   = 130;
  memcpy(txData.mac, peerInfo.peer_addr, 6);


  Serial.print("X: ");
  Serial.print(txData.x_axis);
  Serial.print("\tY: ");
  Serial.println(txData.y_axis);
  // Serial.print("\tPot: ");
  // Serial.print(txData.pot_1);
  // Serial.print("\tSw1: ");
  // Serial.print(txData.sw_1);
  // Serial.print("\tSw2: ");
  // Serial.println(txData.sw_2);
  

  // esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
  esp_now_send(peerInfo.peer_addr, (uint8_t *) &txData, sizeof(txData));
}

// gpio code start
void init_gpio(){
  pinMode(g_x, INPUT);
  pinMode(g_y, INPUT);
  pinMode(pot, INPUT);
  pinMode(switch_1  , INPUT_PULLUP);
  pinMode(switch_2_up, INPUT_PULLUP);
  pinMode(switch_2_down, INPUT_PULLUP);
  pinMode(switch_3  , INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(vbat, INPUT);
  delay(150);
}
// gpio code end

void setup() {  

  Serial.begin(115200);
  // while(!Serial){
  //   ;
  // }
  init_led();


  init_eeprom();
  init_gpio();



  if (get_button_B()) {
    state = BINDING;
  } else {
    if (EEPROM_DATA.binding_status == 1) {
      state = SENDING;
    } else {
      state = BINDING;
    }
  }

  init_data_structures();
  init_esp_now();
  if(EEPROM_DATA.need_to_calibrate){
    calibrate_joystick();
  }else{
    ch1_offset = EEPROM_DATA.offset_x;
    ch2_offset = EEPROM_DATA.offset_y;
  }

  // Serial.print(" eeprom_structure_version = ");
  // Serial.println(EEPROM_DATA.eeprom_structure_version);
  // Serial.print(" reserved = ");
  // Serial.println(EEPROM_DATA.reserved);
  // Serial.print(" calibration needed = ");
  // Serial.println(EEPROM_DATA.need_to_calibrate);
  // Serial.print(" calib_x_low = ");
  // Serial.println(EEPROM_DATA.calib_x_low);
  // Serial.print(" calib_x_high = ");
  // Serial.println(EEPROM_DATA.calib_x_high);
  // Serial.print(" calib_y_low = ");
  // Serial.println(EEPROM_DATA.calib_y_low);
  // Serial.print(" calib_y_high = ");
  // Serial.println(EEPROM_DATA.calib_y_high);
  // Serial.print(" offset_x = ");
  // Serial.println(EEPROM_DATA.offset_x);
  // Serial.print(" offset_y = ");
  // Serial.println(EEPROM_DATA.offset_y);
  
}

void loop() {


  if(millis()-last_led_update > 250){
    last_led_update = millis();
    update_leds();

    // print all hardware pin values
    
    // Serial.print("g_x: ");
    // Serial.print(analogRead(g_x));
    // Serial.print("\tg_y: ");
    // Serial.print(analogRead(g_y));
    // Serial.print("\tpot: ");
    // Serial.print(analogRead(pot));
    // Serial.print("\tsw1: ");
    // Serial.print(get_switch_1());
    // Serial.print("\tsw2: ");
    // Serial.print(get_switch_2());
    // Serial.print("\tsw3: ");
    // Serial.print(get_switch_3());
    // Serial.print("\tBTN_A: ");
    // Serial.print(get_button_A());
    // Serial.print("\tBTN_B: ");
    // Serial.println(get_button_B());
  }

  switch (state) {
    case SENDING:

      if(!rx_lost && millis()-last_sendtime > 50){
        last_sendtime = millis();
        send_joysitck();
      }else if(rx_lost && millis()-last_sendtime > 1000){
        last_sendtime = millis();
        send_joysitck();
      }
      break;

    case BINDING:      
      binding();
      break;
  }
}


