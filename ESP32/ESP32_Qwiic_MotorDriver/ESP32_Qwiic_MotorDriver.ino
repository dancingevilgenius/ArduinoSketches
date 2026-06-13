//This example drives a robot in left and right arcs, driving in an overall wiggly course.
//  It demonstrates the variable control abilities. When used with a RedBot chassis,
//  each turn is about 90 degrees per drive.
//
//  Pin 8 can be grounded to disable motor movement, for debugging.

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

SCMD myMotorDriver; //This creates the main object of one motor driver and connected peripherals.

#define LEFT_DIR_FW   0
#define LEFT_DIR_RV   1
#define RIGHT_DIR_FW  0
#define RIGHT_DIR_RV  1

#define LEFT_MOTOR 1
#define RIGHT_MOTOR 0

void setup()
{
  delay(1000);
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting Sparkfun QWIIC Motor Driver Test.");

  // Kind of like a switch. Use to halt motor movement (ground)
  pinMode(8, INPUT_PULLUP); 

  setupQwiicMotorDriver();

}

void setupQwiicMotorDriver(){

  Serial.println("\nEntering setupQwiicMotorDriver()");  
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

  // Mini Sumo Cobra
  myMotorDriver.inversionMode(LEFT_MOTOR, 1); //  Works:
  //myMotorDriver.inversionMode(LEFT_MOTOR, 1); //  Works:

  if(false){
    //myMotorDriver.inversionMode(LEFT_MOTOR, 1); //  Works: No
   //myMotorDriver.inversionMode(RIGHT_MOTOR, 1); // Works: Yes
  } else {
    //myMotorDriver.inversionMode(LEFT_MOTOR, 0); //  Works: 
    //myMotorDriver.inversionMode(RIGHT_MOTOR, 1); // Works: 
  }


  while ( myMotorDriver.busy() )
    ; // Do nothing until driver is available

  myMotorDriver.enable(); //Enables the output driver hardware
  Serial.println("Exiting setupQwiicMotorDriver()");
}

void loop()
{
  //pass setDrive() a motor number, direction as 0(call 0 forward) or 1, and level from 0 to 255
  /*
  myMotorDriver.setDrive( LEFT_MOTOR, 0, 0); //Stop motor
  myMotorDriver.setDrive( RIGHT_MOTOR, 0, 0); //Stop motor
  while (digitalRead(8) == 0); //Hold if jumper is placed between pin 8 and ground
  */


  motorsTestForwardAndReverse();

}

//***** Operate the Motor Driver *****//
//  This walks through all 34 motor positions driving them forward and back.
//  It uses .setDrive( motorNum, direction, level ) to drive the motors.
void motorsTestForwardAndReverse(){
  //Smoothly drive both motor FWD up to speed and back (drive level 0 to 255)
  Serial.println("Forward Ramp Up");

  float right_fw_factor = 1.0f;
  float left_fw_factor = 0.94f;
  float right_rv_factor = 0.94f;
  float left_rv_factor = 1.0f;

  for (int i = 0; i < 256; i++)
  {
    myMotorDriver.setDrive( LEFT_MOTOR, LEFT_DIR_FW,  (int)(i * left_fw_factor));
    myMotorDriver.setDrive( RIGHT_MOTOR, RIGHT_DIR_FW, (int)(i * right_fw_factor));
    delay(35);
  }
  Serial.println("Forward Ramp Down");
    for (int i = 255; i >= 0; i--)
    {
      myMotorDriver.setDrive( LEFT_MOTOR, LEFT_DIR_FW, (int)(i * left_fw_factor));
      myMotorDriver.setDrive( RIGHT_MOTOR, RIGHT_DIR_FW, (int)(i * right_fw_factor));
      delay(35);
    }
    
    if(true){
    //Smoothly drive both motors REV up to speed and back
    Serial.println("Reverse Ramp up");
    for (int i = 0; i < 256; i++)
    {
      myMotorDriver.setDrive( LEFT_MOTOR, LEFT_DIR_RV, (int)(i * left_rv_factor));
      myMotorDriver.setDrive( RIGHT_MOTOR, RIGHT_DIR_RV, (int)(i * right_rv_factor));
      delay(15);
    }

    Serial.println("Reverse Ramp Down");
    for (int i = 255; i >= 0; i--)
    {
      myMotorDriver.setDrive( LEFT_MOTOR, LEFT_DIR_RV, (int)(i * left_rv_factor)); 
      myMotorDriver.setDrive( RIGHT_MOTOR, RIGHT_DIR_RV, (int)(i * right_rv_factor));
      delay(15);
    }
  }
  Serial.println("Finished motor test cycle. motorsTestForwardAndReverse()");
}

