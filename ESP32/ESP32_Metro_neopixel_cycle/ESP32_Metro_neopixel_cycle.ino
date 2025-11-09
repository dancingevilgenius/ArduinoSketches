

// Include the Adafruit Neopixel Library 
#include <Adafruit_NeoPixel.h>



// metroPixel takes in both the number of pixels (1, the built-in) and the pin)
Adafruit_NeoPixel metroPixel = Adafruit_NeoPixel(1, PIN_NEOPIXEL);

/* Colors */ 
// note: the max. of colors in these arrays is 220 instead of 255 (super-bright!!)
const int RED[ ] = {155, 0, 0};
const int WHITE[ ] = {155, 155, 155};
const int BLUE[ ] = {0, 0, 255};
const int BLACK [ ] = {0, 0, 0};

void setup() {
  Serial.begin(115200);

  // init. the NeoPixel library 
  metroPixel.begin(); 
}

void loop() {
  // display red on the Metro Express neopixel
  pixelWrite(RED);
  delay(1000);
  // display white on the Metro Express neopixel
  pixelWrite(WHITE);
  delay(1000);
  // display blue on the Metro Express neopixel
  pixelWrite(BLUE);
  delay(1000);
  Serial.println("cycle ends");
  // Sparkle the Neopixel 
  // pixelSparkle();
}

// takes in a pre-defined color (integer array) and sets the pixel to that color
void pixelWrite(const int* color) { 
  metroPixel.setPixelColor(0, metroPixel.Color(color[0],color[1],color[2]));
  // write the pixel color to the Metro's Neopixel
  metroPixel.show(); 

}

// flashes the neopixel on and off rapidly 
void pixelSparkle() { 
  for(int i = 0; i < 5; i++) {
    pixelWrite(BLACK);
    delay(50);
    pixelWrite(WHITE);
    delay(50);
  }
}











