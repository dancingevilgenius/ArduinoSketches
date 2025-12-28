/***************************************************
 HUSKYLENS An Easy-to-use AI Machine Vision Sensor
 <https://www.dfrobot.com/product-1922.html>
 
 ***************************************************
 This example shows the basic function of library for HUSKYLENS via Serial.
 
 ****************************************************/

#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include "SparkFun_Qwiic_Joystick_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_joystick

HUSKYLENS huskylens;
SoftwareSerial mySerial(D1,D0); // RX, TX
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11
void printResult(HUSKYLENSResult result);

// QWIIC i2C Joystick
uint8_t Address = 0x20; //Start address (Default 0x20)
JOYSTICK joystick; //Create instance of this object


void setup() {
    Serial.begin(115200);
    mySerial.begin(9600);
    while (!huskylens.begin(mySerial))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }

    setupJoystick();
}

void setupJoystick(){
  if(joystick.begin(Wire1, Address) == false)
  {
    Serial.println("Joystick does not appear to be connected. Please check wiring. Freezing...");
    while(1);
  }
}

void loop() {
    if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
    else
    {
        Serial.println(F("###########"));
        while (huskylens.available())
        {
            HUSKYLENSResult result = huskylens.read();
            printResult(result);
        }    
    }


    loopJoystick();    
}

void loopJoystick(){
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

    if  (Y > 575)
    {
        Serial.println("U " + String(Y));
    }
    else if (Y < 450)
    {
        Serial.println("D " + String(Y));
    } else {
      
    }

    if (B == 0)
    {
        Serial.println("Button");
    }


  delay(200);

}

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");
    }
}