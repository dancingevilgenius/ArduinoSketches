#include <Adafruit_NeoPixel.h>


//#define PIN_NEOPIXEL 8 - Already defined in Feather support code
#define NUMPIXELS 1 // The board has one built-in NeoPixel

// Initialize the NeoPixel strip object
Adafruit_NeoPixel pixel(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void setup() {
  pixel.begin(); // Initialize NeoPixel
  pixel.setBrightness(50); // Set brightness (0-255)
}

void loop() {
  // Set the color to Red (R, G, B)
  pixel.setPixelColor(0, pixel.Color(255, 0, 0));
  pixel.show(); // Show the color
  delay(500); // Wait 500ms

  // Turn off
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();
  delay(500); // Wait 500ms

  // Set the color to Green (R, G, B)
  pixel.setPixelColor(0, pixel.Color(0, 255, 0));
  pixel.show(); // Show the color
  delay(500); // Wait 500ms

  // Turn off
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();
  delay(500); // Wait 500ms


  // Set the color to Blue (R, G, B)
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
  pixel.show(); // Show the color
  delay(500); // Wait 500ms

  // Turn off
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();
  delay(500); // Wait 500ms

}
