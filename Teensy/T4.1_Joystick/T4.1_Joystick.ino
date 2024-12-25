/* Basic USB Joystick Example
   Teensy becomes a USB joystick

   You must select Joystick from the "Tools > USB Type" menu

   Pushbuttons should be connected to digital pins 0 and 1.
   Wire each button between the digital pin and ground.
   Potentiometers should be connected to analog inputs 0 to 1.

   This example code is in the public domain.
*/
#include "Joystick.h"

void setup() {
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);

  Serial.begin(9600);  
}

void loop() {
  int x = 0;
  int y = 0;
  // read analog inputs and set X-Y position
  x = analogRead(0);
  y = analogRead(1);
  Joystick.X(x);
  Joystick.Y(y);

  // read the digital inputs and set the buttons
  Joystick.button(1, digitalRead(0));
  Joystick.button(2, digitalRead(1));

  Serial.println("x:" + x);
  Serial.println("y:" + y);
  // a brief delay, so this runs 20 times per second
  delay(500);
}

