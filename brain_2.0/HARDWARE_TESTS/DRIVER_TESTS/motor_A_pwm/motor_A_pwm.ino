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



void init_io(){
  pinMode(CHIP_SEL,OUTPUT);
  pinMode(FAULT, INPUT);
  pinMode(SLEEP, OUTPUT);
  digitalWrite(SLEEP, HIGH); 
}

uint8_t receiving1 = 0;
uint8_t receiving2 = 0;

void init_drv8908(){

  // set HB 3/4 to pwm mode
  digitalWrite(CHIP_SEL, LOW); // pull low to start the SPI transfer
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // min period 200ns
  delayMicroseconds(1); // must be higher than 100ns
  receiving1 = SPI.transfer(0b00001011);  // PWM_CTRL_1
  receiving2 = SPI.transfer(0b00001100);  // HB 3 and 4 to pwm mode
  SPI.endTransaction();
  delayMicroseconds(1); // must be higher than 100ns
  digitalWrite(CHIP_SEL, HIGH); //pull high to indicate end of SPI transfer
  delay(1);

  // set HB 3/4 to pwm channel
  digitalWrite(CHIP_SEL, LOW); // pull low to start the SPI transfer
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // min period 200ns
  delayMicroseconds(1); // must be higher than 100ns
  receiving1 = SPI.transfer(0b00010000);  // 
  receiving2 = SPI.transfer(0b00001001);  // 
  SPI.endTransaction();
  delayMicroseconds(1); // must be higher than 100ns
  digitalWrite(CHIP_SEL, HIGH); //pull high to indicate end of SPI transfer
  delay(1);

  // // set HB 3/4 to pwm channel
  // digitalWrite(CHIP_SEL, LOW); // pull low to start the SPI transfer
  // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // min period 200ns
  // delayMicroseconds(1); // must be higher than 100ns
  // receiving1 = SPI.transfer(0b00010000);  // 
  // receiving2 = SPI.transfer(0b00001001);  // 
  // SPI.endTransaction();
  // delayMicroseconds(1); // must be higher than 100ns
  // digitalWrite(CHIP_SEL, HIGH); //pull high to indicate end of SPI transfer
  // delay(1);

  // set pwm value channel
  digitalWrite(CHIP_SEL, LOW); // pull low to start the SPI transfer
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // min period 200ns
  delayMicroseconds(1); // must be higher than 100ns
  receiving1 = SPI.transfer(0b00010100);  //
  receiving2 = SPI.transfer(0b00111111);  //
  SPI.endTransaction();
  delayMicroseconds(1); // must be higher than 100ns
  digitalWrite(CHIP_SEL, HIGH); //pull high to indicate end of SPI transfer
  delay(1);

  digitalWrite(CHIP_SEL, LOW); //pull low to start the SPI transfer
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); //min period 200ns
  delayMicroseconds(1); //must be higher than 100ns
  receiving1 = SPI.transfer(0b00001000); //select OP_CTRL_1
  // receiving1 = SPI.transfer(0b01000100);
  receiving2 = SPI.transfer(0b01100000); //set all HS enabled
  // receiving2 = SPI.transfer(0b00000000);
  SPI.endTransaction();
  delayMicroseconds(1); //must be higher than 100ns  
  digitalWrite(CHIP_SEL, HIGH); //pull low to start the SPI transfer
}

void setup() {
  Serial.begin(115200);
  // while(!Serial){
  //   delay(100);
  // }

  SPI.begin(SCK, MISO, MOSI, CHIP_SEL);

  delay(100);
  Serial.println("OLA");
  init_io();

  init_drv8908();
}

void loop() {
  Serial.print("fault pin: ");
  Serial.println(digitalRead(FAULT));
  delay(1000);

  // digitalWrite(CHIP_SEL, LOW); //pull low to start the SPI transfer
  // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); //min period 200ns
  // delayMicroseconds(1); //must be higher than 100ns
  // receiving1 = SPI.transfer(0b00001000); //select OP_CTRL_1
  // // receiving1 = SPI.transfer(0b01000100);
  // receiving2 = SPI.transfer(0b01100000); //set all HS enabled
  // // receiving2 = SPI.transfer(0b00000000);
  // SPI.endTransaction();
  // delayMicroseconds(1); //must be higher than 100ns

  digitalWrite(CHIP_SEL, LOW); //pull low to start the SPI transfer
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); //min period 200ns
  delayMicroseconds(1); //must be higher than 100ns
  receiving1 = SPI.transfer(0b01000000);
  receiving2 = SPI.transfer(0b00000000);
  SPI.endTransaction();
  delayMicroseconds(1); //must be higher than 100ns
  digitalWrite(CHIP_SEL, HIGH); //pull high to indicate end of SPI transfer

  Serial.print(receiving1, BIN);
  Serial.print("\t");
  Serial.println(receiving2, BIN);
  delay(1); //must be larger than 600ns
}
