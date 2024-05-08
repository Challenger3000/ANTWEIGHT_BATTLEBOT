void init_led(){
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB
  leds[0] = CRGB(255, 255, 255);
  FastLED.show();
  delay(10);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
}

void led_color(uint8_t red, uint8_t green, uint8_t blue){
  leds[0] = CRGB(green, red, blue);
  FastLED.show();
}

void led_update(){
  if(millis() - last_led_update < 100){
    return;
  }else{
    last_led_update = millis();
    if(drv8908_overcurrent && millis() % 500 < 250){
      led_color(10, 0, 0);
      return;
    }
    switch (led_state){
      case RX_RECEIVING:
        led_color(0, 10, 0);
        break;
      case RX_LOST:
        led_color(10, 0, 0);
        break;
    }
  }
}