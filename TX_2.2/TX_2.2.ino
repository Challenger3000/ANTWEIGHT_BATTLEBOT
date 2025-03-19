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
#define vbat 8

// rev 1
// #define BTN_A 9
// #define BTN_B 21

// rev 2
#define BTN_A 21
#define BTN_B 9

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
unsigned long last_list_print = 0;

void setup() {
  // Serial.begin(115200);
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
  init_joystick();
}

void loop() {
  if(millis()-last_led_update > 25){
    last_led_update = millis();
    update_leds();

    // print all hardware pin values
    
    // Serial.print("x: ");
    // Serial.print(mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095));
    // Serial.print("\ty: ");
    // Serial.print(mapWithMidpoint(constrain(analogRead(g_y),EEPROM_DATA.calib_y_low,EEPROM_DATA.calib_y_high), EEPROM_DATA.calib_y_low, ch2_offset, EEPROM_DATA.calib_y_high, 0, 4095));
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
        send_data();
      }else if(rx_lost && millis()-last_sendtime > 1000){
        last_sendtime = millis();
        send_data();
      }
      break;

    case BINDING:      
      binding();
      break;
  }
}


