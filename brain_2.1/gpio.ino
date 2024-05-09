void init_gpio(){
  pinMode(BUTTON, INPUT);
  pinMode(VSENSE, INPUT);
}

void update_gpio(){
  if(millis() - last_gpio_update > 100){
    last_gpio_update = millis();
    send_voltage_telemety();
    // delay(500);
    if(!digitalRead(BUTTON)){
      switch_wireles_mode();
    }
    // imu_print();
    // float v_bat = 0.0067441860 * (float)analogRead(VSENSE);
    // Serial.print("V_bat: ");
    // Serial.print(v_bat);
    // Serial.print("\tV_cell: ");
    // Serial.println(v_bat/3.0);
  }
}
