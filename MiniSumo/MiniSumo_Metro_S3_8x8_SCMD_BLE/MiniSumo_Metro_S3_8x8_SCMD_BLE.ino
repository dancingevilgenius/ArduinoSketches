/*!
 * @file get8_8Data.ino
 * @brief This is a demo for retrieving all TOF data. Running this demo will allow you to get all TOF data.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [tangjie](jie.tang@dfrobot.com)
 * @version  V1.0
 * @date  2025-04-03
 * @url https://github.com/DFRobot/DFRobot_MatrixLidar
 */

#include "DFRobot_MatrixLidar.h"

// These are for the Sparkfun QWIIC Motor Driver
#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"


DFRobot_MatrixLidar_I2C tof(0x33);
uint16_t buf[64];

#define MAX_DIST_MM 770
#define ROW_EDGE_DETECTION 7
#define EDGE_DETECTION_THRESHOLD_MM 80

 bool edgeHit[8] = {false, false, false, false, false, false, false, false};

// For Serical Controlled Motor Driver (SCMD) also known as th Sparkfun QWIIC Motor Driver
SCMD myMotorDriver; //This creates the main object of one motor driver and connected peripherals.
#define DIR_FW  0
#define DIR_RV  1
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1


void setup(void){
  Serial.begin(115200);

  setup8x8();
  setupQwiicMotorDriver();


  Serial.println("init success");
}

void setup8x8() {
  while(tof.begin() != 0){
    Serial.println("begin error !!!!!");
  }
  Serial.println("begin success");
  //config matrix mode
  while(tof.setRangingMode(eMatrix_8X8) != 0){
    Serial.println("init error !!!!!");
    delay(1000);
  }

  Serial.println("setup8x8() completed.");
}

void loop8x8(){

  //printRaw8x8();
  bool displayRawScores = false;
  printEdgeDetected(displayRawScores);
}

void clearEdgeHits(){

    for(uint8_t j = 0; j < 8; j++){
      edgeHit[j] = false;
    }

}

void printEdgeDetected(bool displayRawScores){

  clearEdgeHits();

  tof.getAllData(buf);
  int val = -1;
  for(uint8_t i = 0; i < 8; i++){
    if(i == ROW_EDGE_DETECTION ){
      Serial.print("Edge Hit:\t");
      for(uint8_t j = 0; j < 8; j++){
        val = buf[i * 8 + j];
        if(val > 120){
          if(displayRawScores){
            Serial.printf("%04d\t", val);
          } else {
            Serial.printf("X\t");
          }
          edgeHit[j] = true;
        } else {
          if(displayRawScores){
            Serial.printf(" \t");
          } else {
            Serial.printf(" \t");
          }
        }
      }
      Serial.println("");
    }
  }

}


void printRaw8x8(){
  tof.getAllData(buf);
  int val = -1;
  for(uint8_t i = 0; i < 8; i++){
    Serial.print("Y");
    Serial.print(i);
    Serial.print(":\t");
    for(uint8_t j = 0; j < 8; j++){
      val = buf[i * 8 + j];
      if(val < MAX_DIST_MM){
        Serial.printf("%04d\t", val);
      } else {
        Serial.printf("    \t", val);
      }
    }
    Serial.println("");
  }
  Serial.println("------------------------------");  
}


void loop(void){

  // Matrix
  loop8x8();

  //motorsTestForwardAndReverse();


  delay(50);
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

}

//***** Operate the Motor Driver *****//
//  This walks through all 34 motor positions driving them forward and back.
//  It uses .setDrive( motorNum, direction, level ) to drive the motors.
void motorsTestForwardAndReverse(){
  //Smoothly drive both motor FWD up to speed and back (drive level 0 to 255)
  for (int i = 0; i < 256; i++)
  {
    myMotorDriver.setDrive( LEFT_MOTOR, DIR_FW, i);
    myMotorDriver.setDrive( RIGHT_MOTOR, DIR_FW, 20 + (i / 2));
    delay(5);
  }
  for (int i = 255; i >= 0; i--)
  {
    myMotorDriver.setDrive( LEFT_MOTOR, DIR_FW, i);
    myMotorDriver.setDrive( RIGHT_MOTOR, DIR_FW, 20 + (i / 2));
    delay(5);
  }
  
  //Smoothly drive both motors REV up to speed and back
  for (int i = 0; i < 256; i++)
  {
    myMotorDriver.setDrive( LEFT_MOTOR, DIR_RV, 20 + (i / 2));
    myMotorDriver.setDrive( RIGHT_MOTOR, DIR_RV, i);
    delay(5);
  }
  for (int i = 255; i >= 0; i--)
  {
    myMotorDriver.setDrive( LEFT_MOTOR, DIR_RV, 20 + (i / 2));
    myMotorDriver.setDrive( RIGHT_MOTOR, DIR_RV, i);
    delay(5);
  }
  
}



