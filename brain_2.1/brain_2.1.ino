#include "variables.h"
#include "configuration.h"

void setup() {  
  Serial.begin(115200);
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
  led_update();                   // updates the led color based current status
  update_gpio();                  // reads the button and sends the voltage telemetry
  update_filter();                // filters imu data
  update_pid();                   // calculates the PID output
  driving_logic();                // mixes received data and sends it to the motor driver
}
