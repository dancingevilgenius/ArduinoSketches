/***************************************************
 HUSKYLENS An Easy-to-use AI Machine Vision Sensor
 <https://www.dfrobot.com/product-1922.html>
 
 ***************************************************
 This example shows the basic function of library for HUSKYLENS via Serial.
 
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
#include "SoftwareSerial.h"

#define PIN_RX 10
#define PIN_TX 11

// HUSKYLENS coordinate system
#define MIN_X 0
#define MAX_X 320
#define MIN_Y 0
#define MAX_Y 240


HUSKYLENS huskylens;
SoftwareSerial huskySerial(PIN_RX, PIN_TX); // RX, TX
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11
void printResult(HUSKYLENSResult result);

void setup() {
    Serial.begin(115200);
    huskySerial.begin(9600);
    while (!huskylens.begin(huskySerial))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
}

void loop() {
    if (!huskylens.request()) {
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
        //Serial.println(F("###########"));
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