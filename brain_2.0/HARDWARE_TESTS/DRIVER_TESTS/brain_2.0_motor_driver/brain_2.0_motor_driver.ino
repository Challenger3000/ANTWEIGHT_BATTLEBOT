
// motor driver 
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
  A1_BREAK    = 0b00000000, A2_BREAK    = 0b00000000,

  C1_FORWARD  = 0b00000000, C2_FORWARD  = 0b00011000,
  C1_BACKWARD = 0b00000000, C2_BACKWARD = 0b00100100,
  C1_COAST    = 0b00000000, C2_COAST    = 0b00000000,
  C1_BREAK    = 0b00000000, C2_BREAK    = 0b00000000,
  
  B1_FORWARD  = 0b00000001, B2_FORWARD  = 0b00000010,
  B1_BACKWARD = 0b00000010, B2_BACKWARD = 0b00000001,
  B1_COAST    = 0b00000000, B2_COAST    = 0b00000000,
  B1_BREAK    = 0b00000000, B2_BREAK    = 0b00000000,

  D1_FORWARD  = 0b00000100, D2_FORWARD  = 0b10000000,
  D1_BACKWARD = 0b00001000, D2_BACKWARD = 0b01000000,
  D1_COAST    = 0b00000000, D2_COAST    = 0b00000000,
  D1_BREAK    = 0b00000000, D2_BREAK    = 0b00000000,
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

    // write_register_drv8908(OP_CTRL_1, 0b01100000);        // enable 3/4 hafbridges (motor A forward)
    // write_register_drv8908(OP_CTRL_2, 0b00011000);        // enable 6/7 hafbridges (motor C forward)

    write_register_drv8908(PWM_CTRL_2, 0x00);  // enable pwm generation
    motors_initialised = true;
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

  init_drv8908(MOTOR_LAYOUT);
  Serial.println("Starting...\n");
  Serial.println("Running...");
  Serial.print("motor status: ");
  read_drv8908_status();
  
    write_register_drv8908(PWM_DUTY_1, 50);                // sets motor duty cycle
}

void loop() {
  delay(1000);
  
  Serial.println("FORWARD");
  Serial.println();
  // write_register_drv8908(OP_CTRL_2, C2_FORWARD);
  drive_motor_A(FORWARD, 255);
  drive_motor_B(FORWARD, 255);
  read_drv8908_status();



  delay(1000);
  Serial.println("BACKWARD");
  Serial.println();
  // write_register_drv8908(OP_CTRL_2, C2_BACKWARD);
  drive_motor_A(BACKWARD, 255);
  drive_motor_B(BACKWARD, 255);
  read_drv8908_status();
}
