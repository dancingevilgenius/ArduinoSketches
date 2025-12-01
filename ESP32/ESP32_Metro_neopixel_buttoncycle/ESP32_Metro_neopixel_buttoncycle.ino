

// Include the Adafruit Neopixel Library 
#include <Adafruit_NeoPixel.h>



// metroPixel takes in both the number of pixels (1, the built-in) and the pin)
Adafruit_NeoPixel builtinNeopixel = Adafruit_NeoPixel(1, PIN_NEOPIXEL);

/* Colors */ 
// note: the max. of colors in these arrays is 220 instead of 255 (super-bright!!)
const int RED[ ] = {155, 0, 0};
const int WHITE[ ] = {155, 155, 155};
const int BLUE[ ] = {0, 0, 255};
const int GREEN[ ] = {0, 155, 0};
const int BLACK [ ] = {0, 0, 0};

#define BUTTON_PIN   7
boolean oldState = HIGH;
int     mode     = 0;    // Currently-active animation mode, 0-9


void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // init. the NeoPixel library 
  builtinNeopixel.begin(); 
  delay(1000);
  Serial.println("setup() for button cycle test complete.");
}

void loop() {
   // Get current button state.
  boolean newState = digitalRead(BUTTON_PIN);
  
  if((newState == LOW) && (oldState == HIGH)) {
    // Short delay to debounce button.
    delay(20);
    Serial.println("change.");
    oldState = newState;
    // Check if button is still low after debounce.
    newState = digitalRead(BUTTON_PIN);
    if(newState == LOW) {      // Yes, still low
      if(++mode > 2) mode = 0; // Advance to next mode, wrap around after #8
      switch(mode) {           // Start the new animation...
        case 0:
          pixelWrite(BLACK);
          break;
        case 1:
          pixelWrite(RED);
          break;
        case 2:
          pixelWrite(BLUE);
          break;
        case 3:
          pixelWrite(GREEN);
          break;
      }
    }
  }

  delay(100); 
}

void loopCycle(){
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

}

// takes in a pre-defined color (integer array) and sets the pixel to that color
void pixelWrite(const int* color) { 
  builtinNeopixel.setPixelColor(0, builtinNeopixel.Color(color[0],color[1],color[2]));
  // write the pixel color to the Metro's Neopixel
  builtinNeopixel.show(); 

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











