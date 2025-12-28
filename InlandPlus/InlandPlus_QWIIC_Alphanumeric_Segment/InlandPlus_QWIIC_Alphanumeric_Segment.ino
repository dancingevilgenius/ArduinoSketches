/*******************************************************************************************
 * This example tests illuminating individual segments of the display. Pass in the segment
 * and digit you wish to illuminate to illuminateSegement().
 * 
 * Priyanka Makin @ SparkFun Electronics
 * Original Creation Date: January 31, 2020
 * 
 * SparkFun labored with love to create this code. Feel like supporting open source hardware?
 * Buy a board from SparkFun! https://www.sparkfun.com/products/16391
 * 
 * This code is Lemonadeware; if you see me (or any other SparkFun employee) at the 
 * local, and you've found our code helpful, please buy us a round!
 * 
 * Hardware Connections:
 * Attach Red Board to computer using micro-B USB cable.
 * Attach Qwiic Alphanumeric board to Red Board using Qwiic cable.
 * 
 * Distributed as-is; no warranty is given.
 *****************************************************************************************/
#include <Wire.h>

#include <SparkFun_Alphanumeric_Display.h>  //Click here to get the library: http://librarymanager/All#SparkFun_Qwiic_Alphanumeric_Display by SparkFun

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


HT16K33 display;

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun Qwiic Alphanumeric - Example 2: Turn On One Segment");
  delay(1200);
  Wire.begin(0x70); //Join I2C bus

  //check if display will acknowledge
  if (display.begin() == false)
  {
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }
  display.clear(); 
  Serial.println("Display acknowledged.");




  //display.illuminateSegment('A', 0); // Top Horizontal
  //display.illuminateSegment('B', 0); // Top Right Vertical
  //display.illuminateSegment('C', 0); // Bottom Right Vertical
  //display.illuminateSegment('D', 0); // Bottom Horizontal
  //display.illuminateSegment('E', 0); // Bottom Left Vertical
  //display.illuminateSegment('F', 0); // Top Left Vertical
  //display.illuminateSegment('G', 0); // WW Dir
  //display.illuminateSegment('H', 0); // NW Dir
  //display.illuminateSegment('I', 0); // EE Dir
  //display.illuminateSegment('J', 0); // NN Dir
  //display.illuminateSegment('K', 0); // NE Dir
  //display.illuminateSegment('L', 1); // SE Dir
  //display.illuminateSegment('M', 1); // SS Dir
  //display.illuminateSegment('N', 0); // SW Dir

  /*
  display.illuminateSegment('A', 0);
  display.illuminateSegment('B', 0);
  display.illuminateSegment('C', 0);
  display.illuminateSegment('D', 0);
  display.illuminateSegment('E', 0);
  display.illuminateSegment('F', 0);

  display.illuminateSegment('G', 1);
  display.illuminateSegment('I', 1);
  display.illuminateSegment('J', 1);
  display.illuminateSegment('M', 1);

  display.illuminateSegment('H', 2);
  display.illuminateSegment('N', 2);
  display.illuminateSegment('K', 2);
  display.illuminateSegment('L', 2);
  */

  // Outer segments that when all lit look like zero or letter 'O'
  display.illuminateSegment(HOR_TOP, 0);
  display.illuminateSegment(HOR_BOTTOM, 0);
  display.illuminateSegment(VER_TOP_RIGHT, 0);
  display.illuminateSegment(VER_BOTTOM_RIGHT, 0);
  display.illuminateSegment(VER_TOP_LEFT, 0);
  display.illuminateSegment(VER_BOTTOM_LEFT, 0);

  // Inner horizontal and vertical segments that when lit look like plus sign
  display.illuminateSegment(DIR_NN, 1);
  display.illuminateSegment(DIR_EE, 1);
  display.illuminateSegment(DIR_SS, 1);
  display.illuminateSegment(DIR_WW, 1);

  // Inner diagonal segments that when lit look like letter 'X'
  display.illuminateSegment(DIR_NE, 2);
  display.illuminateSegment(DIR_SE, 2);
  display.illuminateSegment(DIR_SW, 2);
  display.illuminateSegment(DIR_NW, 2);



  display.decimalOff();  //Turn decimals on
  display.decimalOn();   //Turn decimals off
  display.colonOff();      //Turn colons on
  display.colonOn();     //Turn colons off


  display.updateDisplay();
}

void loop()
{
}
