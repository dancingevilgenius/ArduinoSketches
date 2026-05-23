#include <Adafruit_NeoPixel.h>

#define LED_PIN    25      // Onboard RGB LED data pin on Pro Micro RP2040
#define LED_COUNT  1       // Only one built-in LED

Adafruit_NeoPixel pixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixel.begin();
  pixel.show();  // Initialize all pixels to 'off'
  pixel.setBrightness(30);
}

void loop() {
  // Red
  pixel.setPixelColor(0, pixel.Color(255, 0, 0));
  pixel.show();
  delay(500);

  // Green
  pixel.setPixelColor(0, pixel.Color(0, 255, 0));
  pixel.show();
  delay(500);

  // Blue
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
  pixel.show();
  delay(500);
}