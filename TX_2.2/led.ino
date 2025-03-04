
void init_led(){
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB
  leds[0] = CRGB(10, 10, 10);
  FastLED.show();
  delay(10);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
}

void led_color(uint8_t led_number,uint8_t red, uint8_t green, uint8_t blue){
  if(led_number < NUM_LEDS && led_number >= 0){
    leds[led_number] = CRGB(green, red, blue);
    FastLED.show();
  }
  FastLED.show();
}
void update_leds(){
  // led 0 is showing battery status
  float battery_voltage = (float)analogRead(vbat)/600.0;
  if(battery_voltage > 4.0){
    led_color(0, 0, 255, 0);
  }else if(battery_voltage > 3.8){
    led_color(0, 255, 255, 0);
  }else if(battery_voltage > 3.5){
    led_color(0, 255, 0, 0);
  }else if(battery_voltage > 3.1){
    if(millis() % 500 < 250){
      led_color(0, 0, 0, 0);
    }else{
      led_color(0, 255, 0, 0);
    }
  }

  // led 1 is showing robots battery status
  // 4.1v - green
  // 3.8v - yellow
  // 3.5v - red
  // 3.1v - red blinking
  if(state == BINDING){
    if(millis() % 500 < 250){
      led_color(1, 0, 0, 255);
    }else{
      led_color(1, 0, 70, 128);
    }
  }else if(rx_lost){
    led_color(1, 0, 0, 0);
  }else if(rxData.volatage > 4.0){
    led_color(1, 0, 255, 0);
  }else if(rxData.volatage > 3.8){
    led_color(1, 255, 255, 0);
  }else if(rxData.volatage > 3.5){
    led_color(1, 255, 0, 0);
  }else if(rxData.volatage > 3.1){
    if(millis() % 500 < 250){
      led_color(1, 0, 0, 0);
    }else{
      led_color(1, 255, 0, 0);
    }
  }

  long x_axis_calibrated = mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095);
  long y_axis_calibrated = mapWithMidpoint(constrain(analogRead(g_y),EEPROM_DATA.calib_y_low,EEPROM_DATA.calib_y_high), EEPROM_DATA.calib_y_low, ch2_offset, EEPROM_DATA.calib_y_high, 0, 4095);

  if( x_axis_calibrated == 0 || x_axis_calibrated == 4095 ){
    led_color(2, 255, 0, 0);
  }else{
    if(x_axis_calibrated > 2047 - 50 && x_axis_calibrated < 2047 + 50){
      led_color(2, 0, 255, 0);
    }else{
      led_color(2, 255, 255, 0);
    }
  }

  if( y_axis_calibrated == 0 || y_axis_calibrated == 4095 ){
    led_color(3, 255, 0, 0);
  }else{
    if(y_axis_calibrated > 2047 - 50 && y_axis_calibrated < 2047 + 50){
      led_color(3, 0, 255, 0);
    }else{
      led_color(3, 255, 255, 0);
    }
  }

  // leds 2 and 3 are free to be used as a general purpouse indicator (currtly used to visually display joystick calibration)  
  // led_color(2, 10, 10, 10);

  // led_color(3, 10, 10, 10);
}