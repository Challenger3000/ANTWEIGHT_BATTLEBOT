

// Code Related to physical pins


int get_pot(){
  return analogRead(pot);
}

uint8_t get_switch_1(){
  return !digitalRead(switch_1);
}

uint8_t get_switch_2(){
  bool sw_pos_up = digitalRead(switch_2_up);
  bool sw_pos_down = digitalRead(switch_2_down);
  if(sw_pos_up && sw_pos_down){
    return 1;
  }
  if(!sw_pos_up && sw_pos_down){
    return 0;
  }
  if(sw_pos_up && !sw_pos_down){
    return 2;
  }
}

uint8_t get_switch_3(){
  return digitalRead(switch_3);
}

uint8_t get_button_A(){
  return !digitalRead(BTN_A);
}

uint8_t get_button_B(){
  return !digitalRead(BTN_B);
}

void init_gpio(){
  pinMode(g_x, INPUT);
  pinMode(g_y, INPUT);
  pinMode(pot, INPUT);
  pinMode(switch_1  , INPUT_PULLUP);
  pinMode(switch_2_up, INPUT_PULLUP);
  pinMode(switch_2_down, INPUT_PULLUP);
  pinMode(switch_3  , INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(vbat, INPUT);
  delay(150);
}