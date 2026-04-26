/***************************************************
 HUSKYLENS An Easy-to-use AI Machine Vision Sensor
 <https://www.dfrobot.com/product-1922.html>
 
 ***************************************************
 This example shows the basic function of library for HUSKYLENS via I2c.
 
 Created 2020-03-13
 By [Angelo qiao](Angelo.qiao@dfrobot.com)
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
 <https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_23>
 2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

#include "HUSKYLENS.h"
#include <Adafruit_NeoPixel.h>

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"
#include "SPI.h"


HUSKYLENS huskylens;
//HUSKYLENS green line >> SDA; blue line >> SCL
void printResult(HUSKYLENSResult result);

// HUSKYLENS coordinate system
#define MIN_X 0
#define MAX_X 320
#define MIN_Y 0
#define MAX_Y 240
#define UNKNOWN_PERCENT  -1.0


// Built-in Neopixel
#define NUMPIXELS 1 // The board has one built-in NeoPixel
// Initialize the NeoPixel strip object
Adafruit_NeoPixel pixel(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

SCMD myMotorDriver; //This creates he main object of one motor driver and connected peripherals.
#define DIR_FW  1
#define DIR_RV  0
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1


// TODO put these vector/target endpoint in a class
float percentTargetX = UNKNOWN_PERCENT;
float percentTargetY = UNKNOWN_PERCENT;


void setup() {
    Serial.begin(115200);
    Wire.begin();

    setupNeopixel();

    while (!huskylens.begin(Wire))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }


  // Kind of like a switch. Use to halt motor movement (ground)
  pinMode(8, INPUT_PULLUP); 

  setupQwiicMotorDriver();

}


void setupQwiicMotorDriver(){
  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face is I2C_MODE 
  myMotorDriver.settings.commInterface = I2C_MODE;

  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern is "1000" (default) on board for address 0x5D

  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;

  //*****initialize the driver get wait for idle*****//
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.print("begin() return word ");
    Serial.print(myMotorDriver.begin(), HEX);
    Serial.print(" : ");
    Serial.println( "ID mismatch, trying again" );    
    delay(500);
  }
  Serial.println( "ID matches 0xA9" );

  //  Check to make sure the driver is done looking for peripherals before beginning
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");
  Serial.println();

  //*****Set application settings and enable driver*****//

  //Uncomment code for motor 0 inversion
  //while( myMotorDriver.busy() );
  //myMotorDriver.inversionMode(0, 1); //invert motor 0

  //Uncomment code for motor 1 inversion
  while ( myMotorDriver.busy() ); //Waits until the SCMD is available.
  myMotorDriver.inversionMode(1, 1); //invert motor 1

  while ( myMotorDriver.busy() )
    ; // Do nothing until driver is available

  myMotorDriver.enable(); //Enables the output driver hardware

    Serial.println("Finished SCMD setup.");
    delay(4000);

    
}


void setupNeopixel(){
  pixel.begin(); // Initialize NeoPixel
  pixel.setBrightness(50); // Set brightness (0-255)

  // Set the color to dim green at first
  pixel.setPixelColor(0, pixel.Color(0, 50, 0));
  pixel.show(); // Show the color
  delay(500); // Wait 500ms

}

void neopixelError(){
  pixel.setPixelColor(0, pixel.Color(50, 0, 0));
  pixel.show(); // Show the color
}

void loop() {
    if (!huskylens.request()) {
        Serial.println(F("Error: Fail to request data from HUSKYLENS, recheck the connection!"));
        neopixelError();
    }
    else if(!huskylens.isLearned()) {
        Serial.println(F("Error: Nothing learned, press learn button on HUSKYLENS to learn one!"));
        neopixelError();
    }
    else if(!huskylens.available()) {
        Serial.println(F("Error: No line detected"));
        neopixelError();
    }
    else
    {
        Serial.println(F("###########"));
        while (huskylens.available())
        {
            HUSKYLENSResult result = huskylens.read();
            printResult(result);
            neopixelFeedback(percentTargetX, percentTargetY);
        }    
    }


    if(percentTargetX < 45){
        myMotorDriver.setDrive( LEFT_MOTOR, DIR_FW, 220);
        myMotorDriver.setDrive( RIGHT_MOTOR, DIR_FW, 200);

    } else if (percentTargetX > 55) {
        myMotorDriver.setDrive( LEFT_MOTOR, DIR_FW, 200);
        myMotorDriver.setDrive( RIGHT_MOTOR, DIR_FW, 220);
    } else {
        myMotorDriver.setDrive( LEFT_MOTOR, DIR_FW, 200);
        myMotorDriver.setDrive( RIGHT_MOTOR, DIR_FW, 200);
    }


}

void neopixelFeedback(float percentX, float percentY){
    if(percentX==UNKNOWN_PERCENT || percentY==UNKNOWN_PERCENT){
        neopixelError();
        return;
    }


    if(percentX >= 40.0 && percentX <= 60.0){
        // Dim Green
        pixel.setPixelColor(0, pixel.Color(0, 150, 0));
        pixel.show(); // Show the color
    } else if(percentX < 40.0){

        // Scaled Yellow
        int saturation = 0;
        saturation = map(percentX, 0, 40, 200, 50);
        
        pixel.setPixelColor(0, pixel.Color(saturation, saturation, 0));
        pixel.show();
    } else if(percentX > 60.0){

        // Scaled Blue
        int saturation = 0;
        saturation = map(percentX, 60.1, 100, 50, 200);        
        pixel.setPixelColor(0, pixel.Color(0, 0, saturation));
        pixel.show();
        
    } else {
        pixel.setPixelColor(0, pixel.Color(0, 0, 0));
        pixel.show(); // Show the color
    }

}




void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println("COMMAND_RETURN_BLOCK is incorrect mode for line following. Exiting.");
        percentTargetX = UNKNOWN_PERCENT;
        percentTargetY = UNKNOWN_PERCENT;
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
        percentTargetX = 100.0*float(endX)/MAX_X;
        percentTargetY = 100.0*float(endY)/MAX_Y;
        Serial.print("pctX:"); Serial.print(percentTargetX); Serial.print(" pctY:"); Serial.println(percentTargetY);
        delay(200);
        //Serial.print("mapY:"); Serial.println(mapY);
    }
    else{
        Serial.println("Object unknown!");
        percentTargetX = UNKNOWN_PERCENT;
        percentTargetY = UNKNOWN_PERCENT;
    }

}
