// eeprom code start
#define EEPROM_SIZE 150
#include <EEPROM.h>
int address = 100;

typedef struct struct_message {
  uint8_t   eeprom_structure_version;
  uint8_t   reserved;
  int32_t   calib_x_low;
  int32_t   calib_x_high;
  int32_t   calib_y_low;
  int32_t   calib_y_high;
  int32_t   offset_x;
  int32_t   offset_y;
} struct_message;
struct_message EEPROM_DATA;
// eeprom code end 

void setup() {
  Serial.begin(115200);
  while(!Serial){
    ;
  }
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(address, EEPROM_DATA);
  
  Serial.print(" eeprom_structure_version = ");
  Serial.print(EEPROM_DATA.eeprom_structure_version);
  Serial.print(" reserved = ");
  Serial.print(EEPROM_DATA.reserved);
  Serial.print(" calib_x_low = ");
  Serial.print(EEPROM_DATA.calib_x_low);
  Serial.print(" calib_x_high = ");
  Serial.print(EEPROM_DATA.calib_x_high);
  Serial.print(" calib_y_low = ");
  Serial.print(EEPROM_DATA.calib_y_low);
  Serial.print(" calib_y_high = ");
  Serial.print(EEPROM_DATA.calib_y_high);
  Serial.print(" offset_x = ");
  Serial.print(EEPROM_DATA.offset_x);
  Serial.print(" offset_y = ");
  Serial.println(EEPROM_DATA.offset_y);

  EEPROM_DATA.eeprom_structure_version++;
  EEPROM_DATA.reserved++;
  EEPROM_DATA.calib_x_high++;
  EEPROM_DATA.calib_x_low++;
  EEPROM_DATA.calib_y_high++;
  EEPROM_DATA.calib_y_low++;
  EEPROM_DATA.offset_x++;
  EEPROM_DATA.offset_y++;


  EEPROM.put(address, EEPROM_DATA);
  EEPROM.commit();
  
}

void loop() {
}
