
// led code start
#include <FastLED.h>
#define NUM_LEDS 1
#define DATA_PIN 1
#define CLOCK_PIN 13
CRGB leds[NUM_LEDS];

void led_init(){
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB
  leds[0] = CRGB(255, 255, 255);
  FastLED.show();
  delay(10);
  leds[0] = CRGB(0, 255, 0);
  FastLED.show();
}

void led_color(uint8_t red, uint8_t green, uint8_t blue){
  leds[0] = CRGB(green, red, blue);
  FastLED.show();
}
// led code end

// imu code start
#include "FastIMU.h"
#include <Wire.h>
#define IMU_ADDRESS 0x6B    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
LSM6DSL IMU;               //Change to the name of any supported IMU! 

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

void init_imu(){
  Wire.setPins(48, 47);
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU calibration & data example");

  delay(2000);
  Serial.println("Keep IMU level.");
  delay(3000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
#endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  led_color(0,10,0);
}

void imu_print(){
  IMU.update();
  IMU.getAccel(&accelData);
  Serial.print(accelData.accelX);
  Serial.print("\t");
  Serial.print(accelData.accelY);
  Serial.print("\t");
  Serial.print(accelData.accelZ);
  Serial.print("\t");
  IMU.getGyro(&gyroData);
  Serial.print(gyroData.gyroX);
  Serial.print("\t");
  Serial.print(gyroData.gyroY);
  Serial.print("\t");
  Serial.print(gyroData.gyroZ);
  Serial.print("\t");
  Serial.println(IMU.getTemp());  
}
// imu code end

// pid stuff code
#include <PID_v1.h>
#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=0.06, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void init_pid(){
  Setpoint = 0.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-128,128);
}

void update_pid(){
  
  IMU.update();
  IMU.getGyro(&gyroData);
  
  Input = (double)gyroData.gyroZ;
  myPID.Compute();
  // Serial.print("yaw: ");
  // Serial.print(gyroData.gyroZ);  
  // Serial.print("\tpid_result: ");
  // Serial.println(Output);

}
// pid code end

// motor driver code start
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
bool motors_initialised = false;

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

uint8_t write_register_drv8908(uint8_t write_register, uint8_t write_data){
  digitalWrite(CHIP_SEL, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  delayMicroseconds(1);
  drv8908_status = SPI.transfer(write_register);
  uint8_t received_data = SPI.transfer(write_data);
  SPI.endTransaction();
  delayMicroseconds(1);  
  digitalWrite(CHIP_SEL, HIGH);
  delay(1);
  return received_data;
}

uint8_t read_register_drv8908(uint8_t read_register){
  digitalWrite(CHIP_SEL, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  delayMicroseconds(1);
  drv8908_status = SPI.transfer(0b01000000 | read_register);
  uint8_t received_data = SPI.transfer(0x00);
  SPI.endTransaction();
  delayMicroseconds(1);  
  digitalWrite(CHIP_SEL, HIGH);
  delay(1);
  return received_data;
}

uint8_t prepare_motor_register(uint8_t motor, uint8_t register){
  uint8_t motor_register = 0;

  return motor_register;
}

void drive_motor_A(uint8_t new_state, uint8_t PWM){
  switch (MOTOR_LAYOUT) {
    case PARALEL_AC_BD:
      switch (new_state) {
        case FORWARD:
          MOTOR_A1_STATE = A1_FORWARD;
          MOTOR_A2_STATE = A2_FORWARD;
          MOTOR_C1_STATE = C1_FORWARD;
          MOTOR_C2_STATE = C2_FORWARD;
          break;
        case BACKWARD:
          MOTOR_A1_STATE = A1_BACKWARD;
          MOTOR_A2_STATE = A2_BACKWARD;
          MOTOR_C1_STATE = C1_BACKWARD;
          MOTOR_C2_STATE = C2_BACKWARD;
          break;
        case COAST: 
          MOTOR_A1_STATE = A1_COAST;
          MOTOR_A2_STATE = A2_COAST;
          MOTOR_C1_STATE = C1_COAST;
          MOTOR_C2_STATE = C2_COAST;
          break;
        case BREAK:
          MOTOR_A1_STATE = A1_BREAK;
          MOTOR_A2_STATE = A2_BREAK;
          MOTOR_C1_STATE = C1_BREAK;
          MOTOR_C2_STATE = C2_BREAK;
          break;
      }
      write_register_drv8908(PWM_CTRL_2, 0xFF);  // disable pwm generation
      write_register_drv8908(OP_CTRL_1, MOTOR_A1_STATE | MOTOR_B1_STATE | MOTOR_C1_STATE | MOTOR_D1_STATE);
      write_register_drv8908(OP_CTRL_2, MOTOR_A2_STATE | MOTOR_B2_STATE | MOTOR_C2_STATE | MOTOR_D2_STATE);
      write_register_drv8908(PWM_DUTY_1, PWM);                // sets motor duty cycle
      write_register_drv8908(PWM_CTRL_2, 0x00);  // enable pwm generation
      break;
    case INDIVIDUAL_A_B_C_D:
    
      break;
  }
}

void drive_motor_B(uint8_t new_state, uint8_t PWM){
  switch (MOTOR_LAYOUT) {
    case PARALEL_AC_BD:
      switch (new_state) {
        case FORWARD:
          MOTOR_B1_STATE = B1_FORWARD;
          MOTOR_B2_STATE = B2_FORWARD;
          MOTOR_D1_STATE = D1_FORWARD;
          MOTOR_D2_STATE = D2_FORWARD;
          break;
        case BACKWARD:
          MOTOR_B1_STATE = B1_BACKWARD;
          MOTOR_B2_STATE = B2_BACKWARD;
          MOTOR_D1_STATE = D1_BACKWARD;
          MOTOR_D2_STATE = D2_BACKWARD;
          break;
        case COAST: 
          MOTOR_B1_STATE = B1_COAST;
          MOTOR_B2_STATE = B2_COAST;
          MOTOR_D1_STATE = D1_COAST;
          MOTOR_D2_STATE = D2_COAST;
          break;
        case BREAK:
          MOTOR_B1_STATE = B1_BREAK;
          MOTOR_B2_STATE = B2_BREAK;
          MOTOR_D1_STATE = D1_BREAK;
          MOTOR_D2_STATE = D2_BREAK;
          break;
      }
      write_register_drv8908(PWM_CTRL_2, 0xFF);  // disable pwm generation
      write_register_drv8908(OP_CTRL_1, MOTOR_A1_STATE | MOTOR_B1_STATE | MOTOR_C1_STATE | MOTOR_D1_STATE);
      write_register_drv8908(OP_CTRL_2, MOTOR_A2_STATE | MOTOR_B2_STATE | MOTOR_C2_STATE | MOTOR_D2_STATE);
      write_register_drv8908(PWM_DUTY_2, PWM);                // sets motor duty cycle
      write_register_drv8908(PWM_CTRL_2, 0x00);  // enable pwm generation
      break;
    case INDIVIDUAL_A_B_C_D:
      break;
  }
}

void init_drv8908(uint8_t config){
  pinMode(CHIP_SEL,OUTPUT);
  pinMode(FAULT, INPUT);
  pinMode(SLEEP, OUTPUT);
  digitalWrite(SLEEP, HIGH); 
  SPI.begin(SCK, MISO, MOSI, CHIP_SEL);
  delay(1);

  switch (config) {
  case PARALEL_AC_BD:
    // general settings for all motors
    write_register_drv8908(PWM_CTRL_1, 0b11111111);       // set all half-bridges to PWM control    
    write_register_drv8908(OLD_CTRL_2, 0b01000000);       // keep driving motors if open load is detected
    write_register_drv8908(PWM_FREQ_CTRL_1, 0b10101010);  // set pwm freq to 200hz for all motors (default: 80, runs rough)
    write_register_drv8908(PWM_FREQ_CTRL_2, 0b10101010);  // 
    // write_register_drv8908(FW_CTRL_1, 0b11111111);     // enables active freewheeling (heats motors, runs rough, meh)
    write_register_drv8908(PWM_CTRL_2, 0xFF);  // disable pwm generation
    // map PWM chanels to halfbridges
    write_register_drv8908(PWM_MAP_CTRL_1, 0b00001001);   // PWM CH2 to OUT_1 and OUT_2
    write_register_drv8908(PWM_MAP_CTRL_2, 0b00000000);   // PWM CH1 to OUT_3 and OUT_4
    write_register_drv8908(PWM_MAP_CTRL_3, 0b00000001);   // PWM CH1 to OUT_6 and PWM CH2 to OUT_5
    write_register_drv8908(PWM_MAP_CTRL_4, 0b00001000);   // PWM CH1 to OUT_7 and PWM CH2 to OUT_8

    write_register_drv8908(PWM_DUTY_1, 0);                // sets motor duty cycle
    write_register_drv8908(PWM_DUTY_2, 0);                // sets motor duty cycle

    write_register_drv8908(PWM_CTRL_2, 0x00);  // enable pwm generation
    motors_initialised = true;
    break;
  case INDIVIDUAL_A_B_C_D:
    // statements
    break;
  }
  Serial.print("motor status: ");
  read_drv8908_status();
}

void read_drv8908_status(){
  digitalWrite(CHIP_SEL, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  delayMicroseconds(1);
  drv8908_status = SPI.transfer(0b01000000);
  uint8_t received_data = SPI.transfer(0b00000000);
  SPI.endTransaction();
  delayMicroseconds(1);
  digitalWrite(CHIP_SEL, HIGH);

  // Serial.print(drv8908_status, BIN);
  // Serial.print("\t");
  // Serial.println(received_data, BIN);
  delay(1);
}
// motor driver code end

// esp_now reciever 
#include <WiFi.h>
#include <esp_now.h>
uint8_t id = 1;
unsigned long last_packet=0;

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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  last_packet=millis();
  Setpoint = map(myData.ch02,0,255,-600,600);
  // Serial.print(myData.mode);
  // Serial.print("\t");
  // Serial.print(myData.id);
  // Serial.print("\t");
  // Serial.print(myData.ch01);
  // Serial.print("\t");
  // Serial.println(myData.ch02);
}

void init_esp_now(){
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}
// esp_now code end

void setup() {
  Serial.begin(115200);
  // while (!Serial) {
  //   ;
  // }
  delay(100);
  Serial.println("Starting...\n");
  Serial.println("Running...");

  led_init();
  init_drv8908(MOTOR_LAYOUT);
  init_imu();
  init_esp_now();
  init_pid();
}

void loop() {
  // imu_print();
  update_pid();
  
  myData.ch01 -= round(Output);
  myData.ch02 += round(Output);
  // if no packets for 100ms assume FS_RC
  if( (millis()-last_packet) > 50 ){
      drive_motor_A(COAST, 0);
      drive_motor_B(COAST, 0);
    // Serial.println("NO SIGNAL");
  }else if(true){
    if(myData.ch01>128){
      drive_motor_A(FORWARD, ((myData.ch01-128)*2)+1);
      // Serial.println("B for ward: ");
    }else if(myData.ch01==128){
      drive_motor_A(COAST, 0);
      // Serial.println("stop");
    }else if(myData.ch01<128){
      drive_motor_A(BACKWARD, ((128-myData.ch01)*2)-1);
      // Serial.println("A backward: ");
    }

    if(myData.ch02>128){
      drive_motor_B(BACKWARD, ((myData.ch02-128)*2)+1);
      // Serial.println("B for ward: ");
    }else if(myData.ch02==128){
      drive_motor_B(COAST, 0);
      // Serial.println("stop");
    }else if(myData.ch02<128){
      drive_motor_B(FORWARD, ((128-myData.ch02)*2)-1);
      // Serial.println("B backward: ");
    }      
  }
}
