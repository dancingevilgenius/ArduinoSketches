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
#include "Arduino.h"
#include "DFRobot_MatrixLidar.h"

// Start for DFRobot MatrixLidar ----------------
DFRobot_MatrixLidar_I2C tof(0x33);
uint16_t buf[64];
// End for DFRobot MatrixLidar ----------------

void setup(void){
  Serial.begin(115200);

  delay(1000); // Delay to Serial initialize before printing stuff.

  setupMatrixLidar();

  Serial.println("setupComplete()");
}

void setupMatrixLidar(){
  while(tof.begin() != 0){
    Serial.println("DFR MatrixLidar not found");
    delay(500);
  }
  //config matrix mode
  while(tof.setRangingMode(eMatrix_8X8) != 0){
    Serial.println("init error !!!!!");
    delay(1000);
  }

  Serial.println("DFR MatrixLidar found! Starting readings in 3 seconds");
}

void loop(void){
  //loopMatrixLidar();
  //loopMatrixMiniSumoRaw();
  loopMiniSumoEdge();
  delay(70);
}

// 1. Ignore value 4000  (indeterminate)
// 1. Ignore value > 770  (size of sumo ring)
void loopMiniSumoEdge(){
  tof.getAllData(buf);
  int val = -1;
  bool edge_hit = false;
  for(uint8_t i = 0; i < 8; i++){
    if(i == 7){

      edge_hit = false;
      for(uint8_t j = 0; j < 8; j++){
        val = buf[i * 8 + j];
        if(val == 4000 || val > (770 - 200)){
          // Do nothing
        } else {
          if(val > 150){
            edge_hit = true;
          }
        }
      }

      if(edge_hit){
        Serial.print("Y");
        Serial.print(i);    
        Serial.print(":\t");
        for(uint8_t j = 0; j < 8; j++){
          val = buf[i * 8 + j];
          Serial.print("\t");
          if(val == 4000 || val > (770 - 200)){
            Serial.print("    ");
          } else {
            if(val > 150){
              Serial.print("EDG0");
            } else {
              Serial.print("    ");
            }            
          }
        }
        Serial.println("");
        Serial.println("------------------------------");

      }


    }
  }
}




// 1. Ignore value 4000  (indeterminate)
// 1. Ignore value > 770  (size of sumo ring)
void loopMatrixMiniSumoRaw(){
  tof.getAllData(buf);
  int val = -1;
  for(uint8_t i = 0; i < 8; i++){
    Serial.print("Y");
    Serial.print(i);
    Serial.print(":\t");
    for(uint8_t j = 0; j < 8; j++){
      val = buf[i * 8 + j];
      Serial.print("\t");
      if(val == 4000 || val > (770 - 200)){
        Serial.print("    ");
      } else {
        Serial.print(val);
      }

    }
    Serial.println("");
  }
  Serial.println("------------------------------");
}

void loopMatrixLidar(){
  tof.getAllData(buf);
  int val = -1;
  for(uint8_t i = 0; i < 8; i++){
    Serial.print("Y");
    Serial.print(i);
    Serial.print(":\t");
    for(uint8_t j = 0; j < 8; j++){
      val = buf[i * 8 + j];
      //Serial.print("%04d\t");
      Serial.print("\t");
      if(val == 4000){
        Serial.print("    ");
      } else {
        Serial.print(val);
      }

    }
    Serial.println("");
  }
  Serial.println("------------------------------");
}
