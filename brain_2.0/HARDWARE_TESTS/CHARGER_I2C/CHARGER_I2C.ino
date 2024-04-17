#include <I2C_8Bit.h>
#include <I2C_16Bit.h>
#include <Wire.h>

// Put your own I2C address here
#define I2C_ADDR 0x6B

// Your register address, which can be found on your IC's datasheet
#define DATA_REGISTER_ADDR 0x03

uint16_t registerData = 0;
uint16_t registerData_2 = 0;

void setup() {
	Serial.begin(115200);
	I2C_8Bit_begin();
	I2C_16Bit_begin();
  Wire.begin( 6, 5 );
  
	I2C_8Bit_writeToModule(I2C_ADDR, 0x10,B00001000);
	I2C_8Bit_writeToModule(I2C_ADDR, 0xF,B01110000);
	I2C_8Bit_writeToModule(I2C_ADDR, 0xA,B01100000); // 2s
	I2C_16Bit_writeToModule(I2C_ADDR, 0x03,100);
	I2C_16Bit_writeToModule(I2C_ADDR, 0x01,835);
  
  
  
}


void loop() {

	// registerData_2 = I2C_16Bit_readFromModule(I2C_ADDR, 0x03);
	// Serial.print("Charge current: ");
	// Serial.print(registerData_2*10);  
	// Serial.println(" mA");
  // registerData = 0;
  // registerData_2 = 0;
  
	// registerData_2 = I2C_16Bit_readFromModule(I2C_ADDR, 0x01);
	// Serial.print("Charge Voltage: ");
	// Serial.println(registerData_2);
  // registerData = 0;
  // registerData_2 = 0;

  // registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x1B);
	// Serial.print("status?: ");
	// Serial.println(registerData,BIN);
  // registerData = 0;
  // registerData_2 = 0;

  // registerData_2 = I2C_16Bit_readFromModule(I2C_ADDR, 0x06);
	// Serial.print("input current: ");
	// Serial.println(registerData_2);
  // registerData = 0;
  // registerData_2 = 0;

  // registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0xF);
	// Serial.print("enable chg?: ");
	// Serial.println(registerData,BIN);
  // registerData = 0;
  // registerData_2 = 0;
  
  // registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x1C);
	// Serial.print("vbus stat?: ");
	// Serial.println(registerData,BIN);
  // registerData = 0;
  // registerData_2 = 0;

  // registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x11);
	// Serial.print("input detct?: ");
	// Serial.println(registerData,BIN);
  // registerData = 0;
  // registerData_2 = 0;

  // registerData_2 = I2C_16Bit_readFromModule(I2C_ADDR, 0x19);
	// Serial.print("inputcurrent limmit: ");
	// Serial.println(registerData_2);
  // registerData = 0;
  // registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x1B);
	Serial.print("CHARGRGER STATUS 0: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x1C);
	Serial.print("CHARGRGER STATUS 1: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x1D);
	Serial.print("CHARGRGER STATUS 2: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x1E);
	Serial.print("CHARGRGER STATUS 3: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x1F);
	Serial.print("CHARGRGER STATUS 4: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x20);
	Serial.print("FAIL STATUS 0: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x21);
	Serial.print("FAIL STATUS 1: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x26);
	Serial.print("fault flag 0: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x27);
	Serial.print("fault flag 1: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x43);
	Serial.print("43: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x45);
	Serial.print("45: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0x0);
	Serial.print("0: ");
	Serial.println(2500+(registerData*250));
  registerData = 0;
  registerData_2 = 0;

  registerData = I2C_8Bit_readFromModule(I2C_ADDR, 0xA);
	Serial.print("A: ");
	Serial.println(registerData,BIN);
  registerData = 0;
  registerData_2 = 0;

  registerData_2 = I2C_16Bit_readFromModule(I2C_ADDR, 0x35);
	Serial.print("V_BUS: ");
	Serial.println(registerData_2);
  registerData = 0;
  registerData_2 = 0;

  registerData_2 = I2C_16Bit_readFromModule(I2C_ADDR, 0x1);
	Serial.print("CHARGE_VOLTAGE: ");
	Serial.println(registerData_2);
  registerData = 0;
  registerData_2 = 0;

  registerData_2 = I2C_16Bit_readFromModule(I2C_ADDR, 0x3);
	Serial.print("CHARGE_CURRENT: ");
	Serial.println(registerData_2);
  registerData = 0;
  registerData_2 = 0;

  registerData_2 = I2C_16Bit_readFromModule(I2C_ADDR, 0x3D);
	Serial.print("vsys: ");
	Serial.println(registerData_2);
	Serial.println(registerData_2,BIN);
  registerData = 0;
  registerData_2 = 0;

	delay(500);
}
