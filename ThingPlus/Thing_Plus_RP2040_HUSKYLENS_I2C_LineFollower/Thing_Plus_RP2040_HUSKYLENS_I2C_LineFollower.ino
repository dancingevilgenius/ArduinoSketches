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

HUSKYLENS huskylens;
//HUSKYLENS green line >> SDA; blue line >> SCL
void printResult(HUSKYLENSResult result);

// HUSKYLENS coordinate system
#define MIN_X 0
#define MAX_X 320
#define MIN_Y 0
#define MAX_Y 240


void setup() {
    Serial.begin(115200);
    Wire1.begin();
    while (!huskylens.begin(Wire1))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
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
}

// TODO put these vector/target endpoint in a class
float percentTargetX = 0.0;
float percentTargetY = 0.0;


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
        percentTargetX = 100.0*float(endX)/MAX_X;
        percentTargetY = 100.0*float(endY)/MAX_Y;
        Serial.print("pctX:"); Serial.print(percentTargetX); Serial.print(" pctY:"); Serial.println(percentTargetY);
        delay(200);
        //Serial.print("mapY:"); Serial.println(mapY);
    }
    else{
        Serial.println("Object unknown!");
    }

}
/*
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
*/