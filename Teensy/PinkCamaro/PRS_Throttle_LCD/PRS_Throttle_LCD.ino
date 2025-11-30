/*
  SerLCD Library - Hello World
  Gaston Williams - August 29, 2018

  This sketch prints "Hello World!" to the LCD
  and shows the time over I2C using the Qwiic system.

  The circuit:
   SparkFun RGB OpenLCD Serial display connected through
   a Sparkfun Qwiic adpater to an Ardruino with a
   Qwiic shield or a Sparkfun Blackboard with Qwiic built in.

  The Qwiic adapter should be attached to the display as follows:
  Display	/ Qwiic Cable Color
 	GND	/	Black
 	RAW	/	Red
 	SDA	/	Blue
 	SCL	/	Yellow

  Note: If you connect directly to a 5V Arduino instead, you *MUST* use
  a level-shifter to convert the i2c voltage levels down to 3.3V for the display.

  This code is based on the LiquidCrystal code originally by David A. Mellis
  and the OpenLCD code by Nathan Seidle at SparkFun.

  License: This example code is in the public domain.

  More info on Qwiic here: https://www.sparkfun.com/qwiic

  AVR-Based Serial Enabled LCDs Hookup Guide
  https://learn.sparkfun.com/tutorials/avr-based-serial-enabled-lcds-hookup-guide
*/

#include <Wire.h>

#include <Joystick.h>     // Analog Joystick
#include <AxisJoystick.h> // Analog Joystick

#include "SparkFun_Qwiic_Joystick_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_joystick


#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD

#define SW_PIN 5
#define VRX_PIN A1
#define VRY_PIN A2

Joystick* joystic; // Analog Joystick

JOYSTICK joystick; // I2c Joystick


SerLCD lcd; // Initialize the library with default I2C address 0x72

void setup() {

  Serial.begin(9600);

  initI2CLCD();

  //initAnalogJoystick();

  initI2cJoystick();

}

void initI2cJoystick(){
  if(joystick.begin() == false)
  {
    Serial.println("Joystick does not appear to be connected. Please check wiring. Freezing...");
    while(1);
  }

}

void initI2CLCD(){
  Wire.begin();

  lcd.begin(Wire); //Set up the LCD for I2C communication

  lcd.setBacklight(255, 255, 255); //Set backlight to bright white
  lcd.setContrast(5); //Set contrast. Lower to 0 for higher contrast.

  lcd.clear(); //Clear the display - this moves the cursor to home position as well
}

void initAnalogJoystick(){
  joystic = new AxisJoystick(SW_PIN, VRX_PIN, VRY_PIN);
}


void loop() {

  //analogJoystck();

  i2cJoystickToLCD();

  // Set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  //lcd.setCursor(0, 1);
  // Print the number of seconds since reset:
  //lcd.print(millis() / 1000);
}


void i2cJoystickToLCD(){

  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  int x = joystick.getHorizontal();
  int y = joystick.getVertical();
  int b = joystick.getButton();

  if(y < 500){
    y = 500;
  }


  if(x > 500){
    x = 500;
  }


  int throttle =  map(y, 500, 1023, 0, 100);
  int amps = map(x, 0, 500, 100, 0);
  int newThrottle = throttle;

  if(amps > 90){
    int newMax = 80 + (100 - amps);
    newThrottle = map(throttle, 0, 100, 0, newMax);
  }


  String strRaw = String("x:") + x + String(" y:") + y + String(" b:") + b;
  String strThrottle = String("T:") + throttle;
  lcd.setCursor(0, 0); // Top Line
  lcd.print(strThrottle);


  String strAmps = String(" A:") + amps;
  lcd.setCursor(5, 0); // Top Line 
  lcd.print(strAmps);

  String strNewThrottle = String("N:") + newThrottle;
  lcd.setCursor(0, 1); // Bottom Line
  lcd.print(strNewThrottle);


  //lcd.setCursor(0, 1); // Bottom Line 
  //lcd.print(strRaw);

  delay(250);

}


void i2cJoystickToSerial(){
  Serial.print("X: ");
  Serial.print(joystick.getHorizontal());

  Serial.print(" Y: ");
  Serial.print(joystick.getVertical());
  
  Serial.print(" Button: ");
  Serial.println(joystick.getButton());

  delay(200);

}

void i2cJoystick(){

}

void anlogJoystck(){
  Serial.print("| SingleRead: " + moveTitle(joystic->singleRead()));
  Serial.print(" | MultipleRead: " + moveTitle(joystic->multipleRead()));
  Serial.print(" | Press: " + String(joystic->isPress()));
  Serial.print(" | Up: " + String(joystic->isUp()));
  Serial.print(" | Down: " + String(joystic->isDown()));
  Serial.print(" | Right: " + String(joystic->isRight()));
  Serial.print(" | Left: " + String(joystic->isLeft()));
  Serial.print(" | VRx: " + String(joystic->readVRx()));
  Serial.print(" | VRy: " + String(joystic->readVRy()));
  Serial.println(" | SW: " + String(joystic->readSW()) + " |");
}

/**
  Return title of the input joystick move.
*/
String moveTitle(const Joystick::Move move) {
  switch (move) {
    case Joystick::Move::NOT:
      return "NOT";
    case Joystick::Move::PRESS:
      return "PRESS";
    case Joystick::Move::UP:
      return "UP";
    case Joystick::Move::DOWN:
      return "DOWN";
    case Joystick::Move::RIGHT:
      return "RIGHT";
    case Joystick::Move::LEFT:
      return "LEFT";
    default:
      return "???";
  }
}

