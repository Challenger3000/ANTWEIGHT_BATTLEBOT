#include "variables.h"

void setup() {  
  Serial.begin(115200);
  Serial.println("Starting...\n");

  // while(!Serial){
  //   ;
  // }
  
  init_gpio();
  init_eeprom();
  led_init();
  init_esp_now_rx();
  init_pid();
  init_servo();
  init_imu();
  init_filter();
  init_drv8908(MOTOR_LAYOUT);
}

void loop() {
  update_gpio();
  update_filter();
  update_pid();
  drive_motors();
}
