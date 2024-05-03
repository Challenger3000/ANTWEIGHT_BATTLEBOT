// general variables start
uint8_t wireles_mode = 0; // 0 - esp_now signal receiver. 1 - wifi web server
double Kp=4.3, Ki=0, Kd=0.05;
bool motors_on = true;
bool new_rx_data = false;
#define GIMBAL_STICK_DEADZONE 50
int motorA_output = 0;
int motorB_output = 0;
int motorC_output = 0;
int motorD_output = 0;
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
  char string[16];
  uint8_t mac[6];
} struct_message;
struct_message myData;
// general variables end


// wifi website variables start
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "TEST_WIFI";
const char* password = "12345678";
double num1, num2, num3;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ROBOT PID TUNER</title>
    <style>
        body, html {
            height: 100%;
            margin: 0;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            font-family: Arial, sans-serif;
        }
        .container {
            width: 100vw; /* Take up full viewport width */
            padding: 20px; /* Padding around the elements */
            box-sizing: border-box;
        }
        input, button {
            display: block;
            width: 100%; /* Full container width */
            padding: 20px; /* Larger touch targets */
            margin: 10px 0; /* Spacing between elements */
            font-size: 2em; /* Larger font size for visibility */
            box-sizing: border-box;
        }
    </style>
</head>
<body>
    <h1>ROBOT PID TUNER</h1>
    <div class="container">
        <div class="label">P</div>
        <input type="number" id="number1" placeholder="P value" maxlength="5">

        <div class="label">I</div>
        <input type="number" id="number2" placeholder="I value" maxlength="5">

        <div class="label">D</div>
        <input type="number" id="number3" placeholder="D value" maxlength="5">

        <button id="button" class="button">Send</button>
    </div>

    <script>
        var gateway = `ws://${window.location.hostname}/ws`;
        var websocket;
        window.addEventListener('load', onLoad);
        function initWebSocket() {
            console.log('Trying to open a WebSocket connection...');
            websocket = new WebSocket(gateway);
            websocket.onopen    = onOpen;
            websocket.onclose   = onClose;
            websocket.onmessage = onMessage;
        }
        function onOpen(event) {
            console.log('Connection opened');
        }
        function onClose(event) {
            console.log('Connection closed');
            setTimeout(initWebSocket, 2000);
        }
        function onMessage(event) {
            var state;
            if (event.data == "1"){
            state = "ON";
            }
            else{
            state = "OFF";
            }
            document.getElementById('state').innerHTML = state;
        }
        function onLoad(event) {
            initWebSocket();
            initButton();
        }
        function initButton() {
            document.getElementById('button').addEventListener('click', send);
        }
        function send(){
            var num1 = document.getElementById('number1').value;
            var num2 = document.getElementById('number2').value;
            var num3 = document.getElementById('number3').value;
            
            // Formatting the message in the ":10:20:30.5:" format
            var message = ":" + num1 + ":" + num2 + ":" + num3 + ":";
            websocket.send(message);
            console.log('Sent: ' + message);
        }

        function request_update(){
            websocket.send("update");
        }
    </script>
</body>
</html>

)rawliteral";
// wifi website variables end 


// eeprom variables start
#define EEPROM_SIZE 200
#define EEPROM_ADDRES 100
#include <EEPROM.h>
typedef struct struct_eeprom {
  uint8_t   eeprom_structure_version;
  uint8_t   reserved;
  double    PID_P;
  double    PID_I;
  double    PID_D;
  uint8_t   binding_status;
  uint8_t   bound_ch;
  uint8_t   bound_mac[6];
  char      encryption_key[16];
} struct_eeprom;
struct_eeprom EEPROM_DATA;
// eeprom variables end 


// led variables start
#include <FastLED.h>
#define NUM_LEDS 1
#define DATA_PIN 1
#define CLOCK_PIN 13
CRGB leds[NUM_LEDS];
// led variables end


// imu variables start
#include "FastIMU.h"
#include <Wire.h>
#define IMU_ADDRESS 0x6B    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
LSM6DSL IMU;                //Change to the name of any supported IMU! 
calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;
// imu variables end


// filter variables start
#include "1euroFilter.h"
static OneEuroFilter f; // not enabled yet, setup has to be called later
// Frequency of your incoming noisy data
// If you are able to provide timestamps, the frequency is automatically determined
#define FREQUENCY   120   // [Hz] 
#define MINCUTOFF   50.0   // [Hz] needs to be tuned according to your application
#define BETA        10.0   // needs to be tuned according to your application

unsigned long start_time;
float filtered_signal;
// filter variables end


// pid variables start
#include <PID_v1.h>
#define PIN_INPUT 0
#define PIN_OUTPUT 3
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
// double Kp=0.06, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// pid variables end


// motor driver variables start
enum DRV8908_MOTOR_CONFIG {
  PARALEL_AC_BD,
  INDIVIDUAL_A_B_C_D
};
enum DRV8908_MOTOR_STATES {
  FORWARD,
  BACKWARD,
  COAST,
  BREAK,
};
enum DRV8908_MOTOR_REGISTER_STATES {
  A1_FORWARD  = 0b01100000, A2_FORWARD  = 0b00000000,
  A1_BACKWARD = 0b10010000, A2_BACKWARD = 0b00000000,
  A1_COAST    = 0b00000000, A2_COAST    = 0b00000000,
  A1_BREAK    = 0b01010000, A2_BREAK    = 0b00000000,

  C1_FORWARD  = 0b00000000, C2_FORWARD  = 0b00011000,
  C1_BACKWARD = 0b00000000, C2_BACKWARD = 0b00100100,
  C1_COAST    = 0b00000000, C2_COAST    = 0b00000000,
  C1_BREAK    = 0b00000000, C2_BREAK    = 0b00010100,
  
  B1_FORWARD  = 0b00000001, B2_FORWARD  = 0b00000010,
  B1_BACKWARD = 0b00000010, B2_BACKWARD = 0b00000001,
  B1_COAST    = 0b00000000, B2_COAST    = 0b00000000,
  B1_BREAK    = 0b00000001, B2_BREAK    = 0b00000001,

  D1_FORWARD  = 0b00000100, D2_FORWARD  = 0b10000000,
  D1_BACKWARD = 0b00001000, D2_BACKWARD = 0b01000000,
  D1_COAST    = 0b00000000, D2_COAST    = 0b00000000,
  D1_BREAK    = 0b00000100, D2_BREAK    = 0b01000000,
};
#include <SPI.h>
#define SCK 14
#define MISO 12
#define MOSI 21
#define CHIP_SEL 13
#define FAULT 9
#define SLEEP 11
uint8_t drv8908_status = 0;

uint8_t MOTOR_A1_STATE = A1_COAST;
uint8_t MOTOR_B1_STATE = B1_COAST;
uint8_t MOTOR_C1_STATE = C1_COAST;
uint8_t MOTOR_D1_STATE = D1_COAST;

uint8_t MOTOR_A2_STATE = A2_COAST;
uint8_t MOTOR_B2_STATE = B2_COAST;
uint8_t MOTOR_C2_STATE = C2_COAST;
uint8_t MOTOR_D2_STATE = D2_COAST;

uint8_t MOTOR_LAYOUT = PARALEL_AC_BD;   // select A-C paralel B-D paralel

enum DRV8908_REGISTERS {
  IC_STAT=0,
  OCP_STAT_1,  OCP_STAT_2,
  UNUSED,
  OLD_STAT_1,  OLD_STAT_2,
  UNUSED2,
  CONFIG_CTRL,
  OP_CTRL_1,  OP_CTRL_2,
  UNUSED3,
  PWM_CTRL_1,  PWM_CTRL_2,
  FW_CTRL_1,
  UNUSED4,
  PWM_MAP_CTRL_1,  PWM_MAP_CTRL_2,  PWM_MAP_CTRL_3,  PWM_MAP_CTRL_4,
  PWM_FREQ_CTRL_1,  PWM_FREQ_CTRL_2,
  PWM_DUTY_1,  PWM_DUTY_2,  PWM_DUTY_3,  PWM_DUTY_4,  PWM_DUTY_5,  PWM_DUTY_6,  PWM_DUTY_7,  PWM_DUTY_8,
  SR_CTRL_1,
  UNUSED5,
  OLD_CTRL_1,  OLD_CTRL_2,  OLD_CTRL_3,  OLD_CTRL_4,  OLD_CTRL_5,  OLD_CTRL_6,
  REGISTERS_COUNT
};
// motor driver variables end


// esp_now reciever
#include "esp_wifi.h"
#include <esp_now.h>
#define binding_ch 14
bool esp_now_is_init = false;
uint8_t current_ch = 0;
uint8_t sending_ch = 2;
uint8_t id = 1;
unsigned long last_receive=0;
bool binding_mode = false;
unsigned long last_sendtime = 0;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;
// esp_now variables end


// gpio variables start
#define BUTTON 4
#define VSENSE 10
// gpio variables end


// servo variables start
#include <ESP32Servo.h>
Servo myservo;
int servo_position = 0;
#define SERVO_1 38
// servo variables end