
// joystick data processing
long mapWithMidpoint(long value, long fromLow, long fromMid, long fromHigh, long toLow, long toHigh) {
  // First, adjust the range around the midpoint
  if(value < fromMid) {
    // Map the lower half
    return map(value, fromLow, fromMid, toLow, toHigh/2);
  } else {
    // Map the upper half
    return map(value, fromMid, fromHigh, toHigh/2, toHigh);
  }
}

void calibrate_joystick(){
  uint32_t warning_blink_start = millis();

  // give warning before joystick midpoint calibration
  while(millis() - warning_blink_start < 5000){
    if(millis() % 500 < 250){      
      leds[0] = CRGB( 0, 255, 0);
      leds[1] = CRGB( 0, 255, 0);
      leds[2] = CRGB( 0, 255, 0);
      leds[3] = CRGB( 0, 255, 0);
    }else{
      leds[0] = CRGB( 255, 0, 0);
      leds[1] = CRGB( 255, 0, 0);
      leds[2] = CRGB( 255, 0, 0);
      leds[3] = CRGB( 255, 0, 0);
    }
    FastLED.show();
  }
  leds[0] = CRGB( 0, 255, 0);
  leds[1] = CRGB( 0, 255, 0);
  leds[2] = CRGB( 0, 255, 0);
  leds[3] = CRGB( 0, 255, 0);

  FastLED.show();
  Serial.println("Calibrating josystick midpoint...");
  ch1_offset = 0;
  ch2_offset = 0;
  for(int i=0; i<5000; i++){
    ch1_offset += analogRead(g_x);
    ch2_offset += analogRead(g_y);
    delay(1);
  }
  ch1_offset = ch1_offset/5000;
  ch2_offset = ch2_offset/5000;
  
  Serial.println("midpoint calibration done\n");
  EEPROM_DATA.need_to_calibrate = 0;
  EEPROM_DATA.offset_x = ch1_offset;
  EEPROM_DATA.offset_y = ch2_offset;

  // calibrate joystick limits
  Serial.println("Calibrating joystick limits...");
  Serial.println("move joystick to all limits for 10sec");
  
  leds[0] = CRGB(255, 255, 0);
  leds[1] = CRGB(255, 255, 0);
  leds[2] = CRGB(255, 255, 0);
  leds[3] = CRGB(255, 255, 0);
  FastLED.show();
  delay(1000);
  unsigned long start_time = millis();
  uint16_t x_low  = 2048;
  uint16_t x_high = 2048;
  uint16_t y_low  = 2048;
  uint16_t y_high = 2048;
  while (millis()-start_time < 10000)
  {
    x_low = min(x_low, analogRead(g_x));
    x_high = max(x_high, analogRead(g_x));
    y_low = min(y_low, analogRead(g_y));
    y_high = max(y_high, analogRead(g_y));

    delay(10);
  }  
  Serial.println("joystick limits calibration done");
  EEPROM_DATA.calib_x_low  = x_low + 100;
  EEPROM_DATA.calib_x_high = x_high - 100;
  EEPROM_DATA.calib_y_low  = y_low + 100;
  EEPROM_DATA.calib_y_high = y_high - 100;

  leds[0] = CRGB(255,0 , 0);
  leds[1] = CRGB(255,0 , 0);
  leds[2] = CRGB(255,0 , 0);
  leds[3] = CRGB(255,0 , 0);
  FastLED.show();
  
  delay(2000);

  EEPROM.put(EEPROM_ADDRES, EEPROM_DATA);
  EEPROM.commit();
}

uint8_t calculate_expo(int rc_in, float expo){

  float scaled_rc = ((float)rc_in/2047)-1;
  scaled_rc = constrain(scaled_rc, -1.0, 1.0);
  float expo_factor = scaled_rc * (1 - expo + (expo * scaled_rc * scaled_rc));
  uint8_t expo_val = (uint8_t)((expo_factor+1)*127.5);
  return expo_val;

}

uint32_t calculate_expo_12_Bit(int rc_in, float expo){

  float scaled_rc = ((float)rc_in/2047)-1;
  scaled_rc = constrain(scaled_rc, -1.0, 1.0);
  float expo_factor = scaled_rc * (1 - expo + (expo * scaled_rc * scaled_rc));
  uint32_t expo_val = (uint32_t)((expo_factor+1)*2048.0);
  return expo_val;

}

void init_joystick(){
  if(EEPROM_DATA.need_to_calibrate){
    calibrate_joystick();
  }else{
    ch1_offset = EEPROM_DATA.offset_x;
    ch2_offset = EEPROM_DATA.offset_y;
    if(analogRead(g_x) > 3000 &&
      analogRead(g_y) < 1000
    ){
      calibrate_joystick();
    }
  }
}