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

DFRobot_MatrixLidar_I2C tof(0x33);
uint16_t buf[64];

#define MAX_DIST_MM 770
#define ROW_EDGE_DETECTION 7
#define EDGE_DETECTION_THRESHOLD_MM 80

void setup(void){
  Serial.begin(115200);

  setup8x8();

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
  printEdgeDetected();
}

void printEdgeDetected(){

  tof.getAllData(buf);
  int val = -1;
  for(uint8_t i = 0; i < 8; i++){
    if(i < ROW_EDGE_DETECTION ){
      continue;
    }
    Serial.print("Bottom Row:\t");
    for(uint8_t j = 0; j < 8; j++){
      val = buf[i * 8 + j];
      if(val > 120){
        Serial.printf("%04d\t", val);
      } else {
        Serial.printf("    \t", val);
      }
    }
    Serial.println("");
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

  delay(100);
}
