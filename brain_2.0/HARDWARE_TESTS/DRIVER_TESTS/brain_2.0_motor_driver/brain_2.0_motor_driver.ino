
// motor driver 
#include <SPI.h>
#define SCK 14
#define MISO 12
#define MOSI 21
#define CHIP_SEL 13
#define FAULT 9
#define SLEEP 11
uint8_t drv8908_status = 0;

enum DRV8908_MOTOR_CONFIG {
  PARALEL_AC_BD,
  INDIVIDUAL_A_B_C_D
};

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


    // set PWM
    write_register_drv8908(PWM_DUTY_1, 50);                // sets motor duty cycle
    write_register_drv8908(PWM_DUTY_2, 50);                // sets motor duty cycle

    write_register_drv8908(OP_CTRL_1, 0b01100101);        // enable 3/4 hafbridges (motor A forward)
    write_register_drv8908(OP_CTRL_2, 0b10011010);        // enable 6/7 hafbridges (motor C forward)
    // write_register_drv8908(OP_CTRL_1, 0b01100000);        // enable 3/4 hafbridges (motor A forward)
    // write_register_drv8908(OP_CTRL_2, 0b00011000);        // enable 6/7 hafbridges (motor C forward)

    // motor C
    // write_register_drv8908(PWM_MAP_CTRL_3, 0b00001000);   // assigns PWM channel 2 to halfbridge 6


    write_register_drv8908(PWM_CTRL_2, 0x00);  // disable pwm generation
    break;
  case INDIVIDUAL_A_B_C_D:
    // statements
    break;
  }
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

  Serial.print(drv8908_status, BIN);
  Serial.print("\t");
  Serial.println(received_data, BIN);
  delay(1);
}

void setup() {
  Serial.begin(115200);
  // while (!Serial) {
  //   ;
  // }
  delay(100);

  init_drv8908(PARALEL_AC_BD);  
  Serial.println("Starting...\n");
}

void loop() {
  delay(1000);
  Serial.println("Running...");
  Serial.print("motor status: ");
  read_drv8908_status();
}
