void init_gpio(){
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(VSENSE, INPUT);
}

void update_gpio(){
  if(millis() - last_gpio_update > 100){
    check_servo_0_before_arming();
    last_gpio_update = millis();
    send_voltage_telemety();
    if(!digitalRead(BUTTON)){
      switch_wireles_mode();
    }
    esp_task_wdt_reset();                 // reset watchdog, to prevent reboot
    // imu_print();
  }
}
