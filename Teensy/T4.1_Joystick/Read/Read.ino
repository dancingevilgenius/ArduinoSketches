#include <Joystick.h>

Joystick joystick(A0, A1, 13);

void setup() {
  Serial.begin(9600);
}

void loop() {
  joystick.print("X: ", "\tY: ", "\tbutton: ", 0, 100);
  Serial.println();
  delay(250);
}
