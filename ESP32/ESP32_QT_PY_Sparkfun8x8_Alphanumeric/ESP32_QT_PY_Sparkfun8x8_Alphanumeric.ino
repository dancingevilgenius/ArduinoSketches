/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <SparkFun_Alphanumeric_Display.h> //Click here to get the library: http://librarymanager/All#SparkFun_Qwiic_Alphanumeric_Display by SparkFun


// Alphanumeric Segmented display
HT16K33 display;

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output
#define NUM_ROWS 8
#define NUM_COLS 8
int dist8x8[NUM_ROWS][NUM_COLS] = {
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1
                                  };

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager and Sparkfun Alphanumeric Display");

  Wire1.begin(); //This resets to 100kHz I2C

  setupAlphanumeric();

  setup8x8();

}

void setupAlphanumeric(){
  bool displaySuccess = display.begin(
          DEFAULT_ADDRESS,
          DEFAULT_NOTHING_ATTACHED,
          DEFAULT_NOTHING_ATTACHED,
          DEFAULT_NOTHING_ATTACHED,
          Wire1); 
  if (!displaySuccess)
  {
    Serial.println("Alphanumeric display did not acknowledge! Freezing.");
    while (1);
  }
  Serial.println("Alphanumeric display initialized.");

  display.print("ALPH");
  delay(2000);
}

void setup8x8(){
  Serial.println("Initializing Sparkfun 8x8 board. This can take up to 10s. Please wait.");
  //Wire1.begin();
  Wire1.setClock(400000); //Sensor has max I2C freq of 400kHz 

  if (myImager.begin((DEFAULT_I2C_ADDR >> 1), Wire1) == false)
  {
    Serial.println(F("Sparkfun TOF 8x8 Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }

  myImager.setResolution(8*8); //Enable all 64 pads
  
  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  myImager.startRanging();

  Serial.print(F("Sparkfun TOF 8x8 Sensor initialized  with imageWidth:"));
  Serial.println(imageWidth);

  // Put this on display.
  display.print("8x8");
  delay(2000);
}


void loop()
{
  loop8x8();

  //loopAlphanumeric();

  delay(500); //Small delay between polling
}

void loopAlphanumeric(){
  // display value of last/bottom row, one of the middle columns
  int d = dist8x8[7][4];


  char buffer[5];
  sprintf(buffer, "%d", d);
  String distStr = buffer;

  display.print(distStr.c_str());
}

void loop8x8(){
  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      int d;
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          if(true){
            d = measurementData.distance_mm[x + y];
            Serial.print("\t");
            Serial.print(d);
            // Save for later
            //dist8x8[y][x] = d;
          }
        }
        if(true){
          Serial.println();
        }
      }
      Serial.println();
    }
  }

}