#include <SPI.h>

// #define DATAOUT 11    //MOSI
// #define DATAIN  12    //MISO
// #define SPICLOCK  13  //sck
// #define CHIPSELECT 10 //ss

#define SCK 14
#define MISO 12
#define MOSI 21
#define CHIP_SEL 13

#define FAULT 9
#define SLEEP 11

enum REGISTERS {
  IC_STAT=0,
  OCP_STAT_1, OCP_STAT_2, OCP_STAT_3,
  OLD_STAT_1, OLD_STAT_2, OLD_STAT_3,
  CONFIG_CTRL,
  OP_CTRL_1, OP_CTRL_2, OP_CTRL_3,
  PWM_CTRL_1, PWM_CTRL_2, FW_CTRL_1,
  FW_CTRL_2,
  PWM_MAP_CTRL_1, PWM_MAP_CTRL_2, PWM_MAP_CTRL_3,
  PWM_FREQ_CTRL,
  PWM_DUTY_CTRL_1, PWM_DUTY_CTRL_2, PWM_DUTY_CTRL_3, PWM_DUTY_CTRL_4,
  SR_CTRL_1, SR_CTRL_2,
  OLD_CTRL_1, OLD_CTRL_2, OLD_CTRL_3, OLD_CTRL_4
};


void init_io(){
  pinMode(CHIP_SEL,OUTPUT);
  pinMode(FAULT, INPUT);
  pinMode(SLEEP, OUTPUT);
  digitalWrite(SLEEP, HIGH); 
}

uint8_t drv8908_status = 0;
uint8_t received_data = 0;

void read_drv8908_status(){
  digitalWrite(CHIP_SEL, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  delayMicroseconds(1);
  drv8908_status = SPI.transfer(0b01000000);
  received_data = SPI.transfer(0b00000000);
  SPI.endTransaction();
  delayMicroseconds(1);
  digitalWrite(CHIP_SEL, HIGH);

  Serial.print(drv8908_status, BIN);
  Serial.print("\t");
  Serial.println(received_data, BIN);
  delay(1);
}

uint8_t write_register_drv8908(uint8_t write_register, uint8_t write_data){

  digitalWrite(CHIP_SEL, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  delayMicroseconds(1);
  // drv8908_status = SPI.transfer(0b01000000 | write_register);
  drv8908_status = SPI.transfer(write_register);
  received_data = SPI.transfer(write_data);
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
  received_data = SPI.transfer(0x00);
  SPI.endTransaction();
  delayMicroseconds(1);  
  digitalWrite(CHIP_SEL, HIGH);
  delay(1);
  return received_data;
}

void init_drv8908(){
  Serial.print("PWM_MAP_CTRL_2: ");
  Serial.println(read_register_drv8908(PWM_MAP_CTRL_2), BIN);

  write_register_drv8908(PWM_CTRL_1, 0b11111111);
  // write_register_drv8908(FW_CTRL_1, 0b11111111); // enables active freewheeling (heats motors meh)
  // write_register_drv8908(PWM_MAP_CTRL_2, 0b00001001);
  // write_register_drv8908(0x13, 0b10101010); // pwm freq to 200hz
  write_register_drv8908(0x16, 50); // duty cycle
  write_register_drv8908(0x20, 0b01000000); // keep driving motors if open load is detected
  write_register_drv8908(OP_CTRL_1, 0b01100000);
  write_register_drv8908(OP_CTRL_2, 0b00000010);
  
  Serial.print("subsequent value: ");
  Serial.println(read_register_drv8908(OP_CTRL_1));
  Serial.print("subsequent value: ");
  Serial.println(read_register_drv8908(OP_CTRL_1));

  
  Serial.print("PWM_MAP_CTRL_2: ");
  Serial.println(read_register_drv8908(PWM_MAP_CTRL_2), BIN);
  Serial.print("PWM_MAP_CTRL_2: ");
  Serial.println(read_register_drv8908(PWM_MAP_CTRL_2), BIN);
}

void setup() {
  Serial.begin(115200);
  // while(!Serial){
  //   delay(100);
  // }

  SPI.begin(SCK, MISO, MOSI, CHIP_SEL);

  delay(100);
  Serial.println("OLA\n");
  init_io();

  init_drv8908();
}

uint8_t motorA = 0;
bool rampup = true;

void loop() {

  write_register_drv8908(0x15, motorA); // write pwm

  if(rampup){
    motorA++;
    if(motorA>254){
      rampup = false;
    }
  }else{
    motorA--;
    if(motorA<1){
      rampup = true;
    }
  }

  delay(5);

  // read_drv8908_status();

}
