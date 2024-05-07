void init_gpio(){
  pinMode(BUTTON, INPUT);
  pinMode(VSENSE, INPUT);
}

void update_gpio(){
  if(!digitalRead(BUTTON)){
    switch_wireles_mode();
    delay(500);
  }
}

  // imu_print();
  // if(Serial){
  // double v_bat = 0.0067441860 * (double)analogRead(VSENSE);
  // Serial.print("V_bat: ");
  // Serial.print(v_bat);
  // Serial.print("\tV_cell: ");
  // Serial.println(v_bat/3.0);
  // }