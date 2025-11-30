/*
     Servo Motor Control using the Arduino Servo Library
           by Dejan, https://howtomechatronics.com
*/

#include <Servo.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Joystick_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_joystick

uint8_t Address = 0x20; //Start address (Default 0x20)

JOYSTICK joystick; //Create instance of this object

Servo myservo;  // create servo object to control a servo
#define STOPPED 1500

void setup() {
  Serial.begin(9600);
  Serial.println("Qwiic Joystick Example");


  // Servo Setup
  myservo.attach(D5,600,2300);  // (pin, min, max)


  // Joystick Setup
  if(joystick.begin(Wire, Address) == false)
  {
    Serial.println("Joystick does not appear to be connected. Please check wiring. Freezing...");
    while(1);
  }  
}

void loop() {
    int X = joystick.getHorizontal();
    int Y = joystick.getVertical();
    int B = joystick.getButton();
    if  (X > 575)
    {
        Serial.println("L " + String(X));
    }
    else if (X < 450)
    {
        Serial.println("R " + String(X));
    }

    if((Y > 333) && (Y < 666 ))
    {
        myservo.write(92);  // tell servo to go to a particular angle
        //myservo.writeMicroseconds(STOPPED);
    }
    else if  (Y > 575)
    {
        Serial.println("U " + String(Y));
        myservo.write(0);  // tell servo to go to a particular angle

    }
    else if (Y < 450)
    {
        Serial.println("D " + String(Y));
        myservo.write(180);  // tell servo to go to a particular angle

    } 
      
    if (B == 0)
    {
        Serial.println("Button");
    }


  delay(200);
  /*
  myservo.write(0);  // tell servo to go to a particular angle
  delay(1000);
  
  myservo.write(90);              
  delay(500); 
  
  myservo.write(135);              
  delay(500);
  
  myservo.write(180);              
  delay(1500);    
  */                 
}