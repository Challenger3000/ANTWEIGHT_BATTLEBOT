// led code start
#include <FastLED.h>
#define NUM_LEDS 1
#define DATA_PIN 1
#define CLOCK_PIN 13
CRGB leds[NUM_LEDS];

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
// led code end

void setup() {
  led_init();
  led_color(255,0,0);
  delay(500);
}

void loop() { 
  leds[0] = CRGB(0, 10, 0);
  FastLED.show();
  delay(500);

  leds[0] = CRGB(10, 0, 0);
  FastLED.show();
  delay(500);

  leds[0] = CRGB(0, 10, 10);
  FastLED.show();
  delay(500);

}
