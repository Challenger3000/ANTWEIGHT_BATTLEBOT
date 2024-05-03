void led_init(){
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