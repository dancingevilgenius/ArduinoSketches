/***************************************************
 HUSKYLENS An Easy-to-use AI Machine Vision Sensor
 <https://www.dfrobot.com/product-1922.html>
 
 ***************************************************
 This example shows the basic function of library for HUSKYLENS via Serial.
 
 ****************************************************/

#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include <SparkFun_Alphanumeric_Display.h>              //Click here to get the library: http://librarymanager/All#SparkFun_Qwiic_Alphanumeric_Display by SparkFun


#define PIN_RX 10
#define PIN_TX 11

// HUSKYLENS coordinate system
#define MIN_X 0
#define MAX_X 320
#define MIN_Y 0
#define MAX_Y 240


HUSKYLENS huskylens;
// Hardware Connection is on the horizontal pins labelled 10,11
// NOT the vertical pin group labelled TX0 RX0 GND 5V
SoftwareSerial mySerial(10,11); // RX, TX
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11
void printResult(HUSKYLENSResult result);


// Alphanumeric Display. 4 characters
HT16K33 display;

#define ALPHANUMERIC_MAX_CHARS 4
// Horizontal Side Segments
#define HOR_TOP 'A'
#define HOR_BOTTOM 'D'

// Vertical Side Segments
#define VER_BOTTOM_RIGHT 'C'
#define VER_TOP_RIGHT 'B'
#define VER_BOTTOM_LEFT 'E'
#define VER_TOP_LEFT 'F'

// Compass Direction Segments on the inside
#define DIR_NN 'J'
#define DIR_NE 'K'
#define DIR_EE 'I'
#define DIR_SE 'L'
#define DIR_SS 'M'
#define DIR_SW 'N'
#define DIR_WW 'G'
#define DIR_NW 'H'




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

    setupAlphanumericDisplay();
}

// Initialization plus a '0' '+' 'X' test pattern
void setupAlphanumericDisplay()
{
  Serial.println("SparkFun Qwiic Alphanumeric - Example 1: Print String");

  Wire.begin(0x70); //Join I2C bus

  
  if (display.begin() == false)
  {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1){
        Serial.println("Alphanumeric Display did not initialize properly.");
        exit(2);
    }
  }
  Serial.println("Display acknowledged.");

    // Outer segments that when all lit look like zero or letter 'O''

    display.clear();
    display.updateDisplay();


    for(int i=0 ; i<ALPHANUMERIC_MAX_CHARS ; i++){
        display.illuminateSegment(HOR_TOP, i);
        display.illuminateSegment(HOR_BOTTOM, i);
        display.illuminateSegment(VER_TOP_RIGHT, i);
        display.illuminateSegment(VER_BOTTOM_RIGHT, i);
        display.illuminateSegment(VER_TOP_LEFT, i);
        display.illuminateSegment(VER_BOTTOM_LEFT, i);
    }
    display.updateDisplay();
    delay(700);

    display.clear();
    display.updateDisplay();
    
    // Inner horizontal and vertical segments that when lit look like plus sign
    for(int i=0 ; i<ALPHANUMERIC_MAX_CHARS ; i++){
        display.illuminateSegment(DIR_NN, i);
        display.illuminateSegment(DIR_EE, i);
        display.illuminateSegment(DIR_SS, i);
        display.illuminateSegment(DIR_WW, i);
    }
    display.updateDisplay();
    delay(700);


    display.clear();
    display.updateDisplay();

    // Inner diagonal segments that when lit look like letter 'X'
    for(int i=0 ; i<ALPHANUMERIC_MAX_CHARS ; i++){
        display.illuminateSegment(DIR_NE, i);
        display.illuminateSegment(DIR_SE, i);
        display.illuminateSegment(DIR_SW, i);
        display.illuminateSegment(DIR_NW, i);
    }    
    display.updateDisplay();
    delay(700);

    display.clear();
    display.updateDisplay();

}




void loop() {
    if (!huskylens.request()){
         Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    }
    else if(!huskylens.isLearned()) {
        Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    }
    else if(!huskylens.available()) {
        Serial.println(F("No block or arrow appears on the screen!"));
    }
    else
    {
        Serial.println(F("###########"));
        while (huskylens.available())
        {
            HUSKYLENSResult result = huskylens.read();
            printResult(result);
        }    
    }


}


void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println("COMMAND_RETURN_BLOCK is incorrect mode for line following. Exiting.");
        exit(0);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        long startX = result.xOrigin;
        long startY = result.yOrigin;
        long endX = result.xTarget;
        long endY = result.yTarget;

        // Change coordinate system so that positive Y is top/up
        endY = map(endY, MAX_Y, 0, 0, MAX_Y);

        //Serial.println(String()+F("Arrow:startX=")+startX+F(",startY=")+startY+F(",endX=")+endX+F(",endY=")+endY+F(",ID=")+result.ID);
        float percentX = 100.0*float(endX)/MAX_X;
        float percentY = 100.0*float(endY)/MAX_Y;
        Serial.print("pctX:"); Serial.print(percentX); Serial.print(" pctY:"); Serial.println(percentY);
        delay(200);
        //Serial.print("mapY:"); Serial.println(mapY);
    }
    else{
        Serial.println("Object unknown!");
    }

}