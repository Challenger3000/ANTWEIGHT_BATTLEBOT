#include "variables.h"
#include "configuration.h"


void setup() {  
  Serial.begin(115200);
  
  // while(!Serial){
  //   ;
  // }
  
  init_gpio();
  init_eeprom();
  init_led();
  init_esp_now_rx();
  init_pid();
  init_servo();
  init_imu();
  init_filter();
  init_drv8908(MOTOR_LAYOUT);
}

void loop() {
  led_update();
  update_gpio();
  update_filter();
  update_pid();
  driving_logic();
}
