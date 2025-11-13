/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital pin 13,
  when pressing a pushbutton attached to pin 2.

  The circuit:
  - LED attached from pin 13 to ground through 220 ohm resistor
  - pushbutton attached to pin 2 from +5V
  - 10K resistor attached to pin 2 from ground

  - Note: on most Arduinos there is already an LED on the board
    attached to pin 13.

  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  https://docs.arduino.cc/built-in-examples/digital/Button/
*/

// Acebott ESP-32-MAX
// not good right now: 1,2,3,4,7,8
// good:  H1/6 H2/5 H3/17 H4/18
#define H1 5
#define H2 16
#define H3 17
#define H4 18
#define H6 19
#define H7 23

#define H7 12 // Want to use H7 for flipper
#define H8 13
#define H9 14
#define H10 25
#define H11 26

#define H16 32

#define BUILT_IN_LED 2  // works on the Acebott

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 15;  // the number of the pushbutton pin
const int ledPin = BUILT_IN_LED;    // the number of the LED pin

// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status

void setup() {
  Serial.begin(115200);
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLDOWN);

  delay(1000);
  Serial.println("setup() completed for reflectance sensor test");
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
    Serial.println("buttonState: 1");
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
    Serial.println("buttonState: 0");
  }

  delay(1200);
}
