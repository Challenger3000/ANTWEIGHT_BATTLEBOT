void init_gpio(){
  pinMode(BUTTON, INPUT);
  pinMode(VSENSE, INPUT);
}

void update_gpio(){
  if(millis() - last_gpio_update > 100){
    last_gpio_update = millis();
    send_voltage_telemety();
    if(!digitalRead(BUTTON)){
      switch_wireles_mode();
    }
    // imu_print();
  }
}
