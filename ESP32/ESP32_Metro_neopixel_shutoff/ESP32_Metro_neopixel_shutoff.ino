

// Include the Adafruit Neopixel Library 
#include <Adafruit_NeoPixel.h>



// metroPixel takes in both the number of pixels (1, the built-in) and the pin)
Adafruit_NeoPixel metroPixel = Adafruit_NeoPixel(1, PIN_NEOPIXEL);

/* Colors */ 
// note: the max. of colors in these arrays is 220 instead of 255 (super-bright!!)
const int BLACK [ ] = {0, 0, 0};

void setup() {
  Serial.begin(115200);

  // init. the NeoPixel library 
  metroPixel.begin();

  pixelWrite(BLACK);

}

void loop() {
  delay(1000);
}

// takes in a pre-defined color (integer array) and sets the pixel to that color
void pixelWrite(const int* color) { 
  metroPixel.setPixelColor(0, metroPixel.Color(color[0],color[1],color[2]));
  // write the pixel color to the Metro's Neopixel
  metroPixel.show(); 

}












