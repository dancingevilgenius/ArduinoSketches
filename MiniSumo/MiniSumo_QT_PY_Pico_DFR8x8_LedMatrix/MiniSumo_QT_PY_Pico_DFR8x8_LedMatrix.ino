//  Mini Sumo
//  Hardware:
//    DFRobot TOF 8x8 Matrix
//    Adafruit 13x9 LED Matrix
//
//  Other Features:
//    Send 8x8 matrix data to web page
//    Load from index_dpad.html file saved on PSRAM.


#include "Arduino.h"
#include "Wire.h"               // I2C communication
#include "DFRobot_MatrixLidar.h"// 8x8 Lidar Matrix
#include <Adafruit_IS31FL3741.h>// 13x9 LED Matrix
#include <WiFi.h>               // Web Server
#include <AsyncTCP.h>           // Web sockets
#include <ESPAsyncWebServer.h>  // Web sockets
#include <LittleFS.h>           // PSRAM local storage
#include <ArduinoJson.h>
#include <map>

// ------------------------------------------------------------
// WiFi Credential Rotation (REORDERED)
// ------------------------------------------------------------
struct WifiCredential {
  const char* ssid;
  const char* password;
};

WifiCredential wifiList[3] = {
  { "TheMandaloriKen", "asdf12346302201111" },
  { "TheMandalorian",  "6302201111" },
  { "STDL5301",        "library30" }
};
String pendingMessage = "";
String pendingSeverity = "info";


Adafruit_IS31FL3741_QT ledmatrix;
// If colors appear wrong on matrix, try invoking constructor like so:
// Adafruit_IS31FL3741_QT ledmatrix(IS3741_RBG);

#define TOF_8x8_NUM_ROWS 8
#define TOF_8x8_NUM_COLS 8
#define X_OFFSET 2        // Screen is 13 rows wide, 8x8 is only 8 wide.
#define D_OPP_MIN 1
#define D_OPP_MAX 30
#define D_EDGE6_MAX 12
#define D_EDGE7_MAX 8

uint8_t matrix[TOF_8x8_NUM_ROWS][TOF_8x8_NUM_COLS] = {
  {0,0, 0, 0, 0, 0, 0 , 0},
  {0,0, 0, 0, 0, 0, 0 , 0},
  {0,0, 0, 0, 0, 0, 0 , 0},
  {0,0, 0, 20, 20, 0, 0 , 0},
  {0,0, 0, 0, 0, 0, 0 , 0},
  {0,0, 0, 0, 0, 0, 0 , 0},
  {10,10, 10, 10, 10, 10, 10 , 10},
  {10,10, 10, 8, 10, 10, 10,10}
};

// 8x8 Text
char text[] = "ADAFRUIT!";   // A message to scroll
int text_x = 1; //ledmatrix.width(); // Initial text position = off right edge
int text_y = 1;
int text_min;                // Pos. where text resets (calc'd later)


// Some boards have just one I2C interface, but some have more...
//TwoWire *WIRE_I2C = &Wire; // Pro Micro ESP32-C3,
TwoWire *WIRE_I2C = &Wire1; // QT PY Pico,



// Start for DFRobot MatrixLidar ----------------
DFRobot_MatrixLidar_I2C tof(0x33, WIRE_I2C);
uint16_t buf[64];
#define INVALID_VAL 4000
#define RING_SIZE_MM 770
#define ROBOT_SIZE_MM 100
#define MAX_DIST 570    // 770 - 100 - 100
// Thing Plus Pro Micro RP2040
//#define SDA_PIN 5 
//#define SCL_PIN 6 
// QT PY Pico
#define SDA_PIN 22 
#define SCL_PIN 23 

// End for DFRobot MatrixLidar ----------------


void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(10);
  }
  delay(2000);

  Serial.println("MiniSumo QT PY Pico LedMatrix and DFR8x8");
  
  connectToWiFi();

  setupI2C();

  setupLedMatrix();

  setupDFR8x8();

}

// ------------------------------------------------------------
// WiFi rotation connect
// ------------------------------------------------------------
bool connectToWiFi() {
  Serial.println("Starting WiFi credential rotation...");

  for (int i = 0; i < 3; i++) {
    Serial.print("Trying SSID: ");
    Serial.println(wifiList[i].ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiList[i].ssid, wifiList[i].password);

    int failCount = 0;
    while (WiFi.status() != WL_CONNECTED && failCount < 20) {
      delay(500);
      Serial.print(".");
      failCount++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected!");
      pendingMessage = String("Connected to ") + wifiList[i].ssid;
      pendingSeverity = "success";

      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      return true;
    }

    Serial.println("\nFailed to connect. Moving to next SSID...");
  }

  pendingMessage = "Failed to connect to any WiFi network";
  pendingSeverity = "error";

  Serial.println("ERROR: Could not connect to ANY WiFi network.");
  return false;
}


void setupI2C(){
  // Pro Micro ESP32-C3 needs the next line.
  //WIRE_I2C->setPins(SDA_PIN, SCL_PIN);
  WIRE_I2C->begin();
}

void setupDFR8x8(){

  
  while(tof.begin() != 0){
    Serial.println("DFR MatrixLidar not found");
    delay(500);
  }
  Serial.println("DFR MatrixLidar found.");


  int count=0;
  //config matrix mode
    Serial.println("DFR MatrixLidar init starting. This may take several seconds");
  while(tof.setRangingMode(eMatrix_8X8) != 0){
    Serial.print(count);
    Serial.print(" ");
    ++count;
    if(count > 10){
      Serial.println("DFR MatrixLidar not initialized. Leaving setupMatrixLidar()");
      return;
    }
    delay(250);
  }

  Serial.println("DFR MatrixLidar initialized!");
}




void setupLedMatrix(){

  if (! ledmatrix.begin(IS3741_ADDR_DEFAULT, WIRE_I2C)) {
    Serial.println("LED 13x9 Matrix not found");
    return;
  }

  Serial.println("LED 13x9 Matrix found!");

  // By default the LED controller communicates over I2C at 400 KHz.
  // Arduino Uno can usually do 800 KHz, and 32-bit microcontrollers 1 MHz.
  //WIRE_I2C->setClock(800000);

  // Set brightness to max and bring controller out of shutdown state
  ledmatrix.setLEDscaling(0xAA); //0xFF
  ledmatrix.setGlobalCurrent(0xCC); //0xFF
  Serial.print("Global current set to: ");
  Serial.println(ledmatrix.getGlobalCurrent());
  ledmatrix.enable(true); // bring out of shutdown

  // Text Init
  ledmatrix.setRotation(0);
  ledmatrix.setTextWrap(false);
  uint16_t w, h;
  int16_t ignore;
  ledmatrix.getTextBounds(text, 0, 0, &ignore, &ignore, &w, &h);
  text_min = -w; // Off left edge this many pixels


  // Set all pixels to black 0x000
  clearLEDMatrix();
  ledMatrixKeyValue("MA", "OK", 1500);
  clearLEDMatrix();
  delay(2000);

}

uint16_t hue_offset = 0;

void loop() {

    loopMiniSumoOpponent();
    //loopSimulateMiniSumo();
    delay(80);
}


// 1. Ignore value 4000  (indeterminate)
// 1. Ignore value > 770  (size of sumo ring)
void loopMiniSumoOpponent(){

  //clearLEDMatrix();
  uint16_t oppColor = ledmatrix.color565(0, 150,0);
  uint16_t edgeColor = ledmatrix.color565(150, 0,0);
  uint16_t edgeWarnColor = ledmatrix.color565(180, 180, 0); // FFDE21

  tof.getAllData(buf);
  int d_mm = -1;
  for(uint8_t y = 0; y < TOF_8x8_NUM_ROWS; y++){
    if(y==3 || y==4){
      for(uint8_t x = 0; x < TOF_8x8_NUM_COLS; x++){
        d_mm = buf[y * 8 + x];
        if(d_mm == INVALID_VAL || d_mm > MAX_DIST){
          ledmatrix.drawPixel(x+X_OFFSET, y, 0);
        } else {
          if(d_mm < 500){
            ledmatrix.drawPixel(x+X_OFFSET, y, oppColor);
          }
        }
      }
    } else if(y==6){
      for(uint8_t x = 0; x < TOF_8x8_NUM_COLS; x++){
        d_mm = buf[y * 8 + x];
        if(d_mm == INVALID_VAL || d_mm > MAX_DIST){
          ledmatrix.drawPixel(x+X_OFFSET, y, 0);
        } else {
          if(d_mm > 200){
            ledmatrix.drawPixel(x+X_OFFSET, y, edgeWarnColor);
          }
        }
      }
    }  else if(y==7){
      for(uint8_t x = 0; x < TOF_8x8_NUM_COLS; x++){
        d_mm = buf[y * 8 + x];
        if(d_mm == INVALID_VAL || d_mm > MAX_DIST){
          ledmatrix.drawPixel(x+X_OFFSET, y, 0);
        } else {
          if(d_mm > 170){
            ledmatrix.drawPixel(x+X_OFFSET, y, edgeColor);
          }
        }
      }
    }


  }
}


void ledMatrixKeyValueColor(String key, String value, uint16_t key_color, uint16_t value_color, int delay_time){

  ledMatrixStringColor(key,   key_color, delay_time);
  ledMatrixStringColor(value, value_color, delay_time);
}


void ledMatrixKeyValue(String key, String value, int delay_time){

  uint16_t color565;
  color565 = ledmatrix.color565(160, 32, 240); // purple
  ledmatrix.setTextColor(color565); // No background color needed

  ledMatrixStringColor(key,   color565, delay_time);
  ledMatrixStringColor(value, color565, delay_time);
}


void ledMatrixStringColor(String s, uint16_t color565, int delay_time){

    ledmatrix.setTextColor(color565); // No background color needed

    ledmatrix.setCursor(text_x, text_y);
    ledmatrix.fill(0); // Fill screen to erase old text
    ledmatrix.print(s); // write the string
    ledmatrix.show(); // Buffered matrix MUST use show() to update!
    delay(delay_time);
}




void ledMatrixString(String s, int delay_time){
    ledmatrix.setCursor(text_x, text_y);
    ledmatrix.fill(0); // Fill screen to erase old text
    ledmatrix.print(s); // write the string
    ledmatrix.show(); // Buffered matrix MUST use show() to update!
    delay(delay_time);
}

void loopSimulateMiniSumo(){
  uint16_t color565;
  int d;
  for (int y=0; y<TOF_8x8_NUM_ROWS ; y++) {
    for (int x=0; x<TOF_8x8_NUM_COLS; x++) {
      d = matrix[y][x];
      if(y<TOF_8x8_NUM_ROWS -2 ){
        
        if(d > D_OPP_MIN && d < D_OPP_MAX){
          color565 = ledmatrix.color565(0, 150,0);
        } else {
          color565 = ledmatrix.color565(0, 0, 0);
        }
        ledmatrix.drawPixel(x+X_OFFSET, y, color565);
      } else {
        if(y == 6){
          if(d > D_EDGE6_MAX){
            color565 = ledmatrix.color565( 100,0, 0);
          } else {
            color565 = ledmatrix.color565( 10,50,50);
          }
        } else if(y==7){
          if(d > D_EDGE7_MAX){
            color565 = ledmatrix.color565( 100,0, 0);
          } else {
            color565 = ledmatrix.color565( 10,50,50);
          }
        }
        ledmatrix.drawPixel(x+X_OFFSET, y, color565);
      }
    }
  }
  delay(2000);

}

void loopReadFromMatrix(){
  uint8_t scale_factor = 18;
  uint16_t color565;
  for (int y=0; y<TOF_8x8_NUM_ROWS ; y++) {
    for (int x=0; x<TOF_8x8_NUM_COLS; x++) {
      if(y<TOF_8x8_NUM_ROWS -2 ){
        if(matrix[y][x] >=20){
          color565 = ledmatrix.color565(0, matrix[y][x],0);
        } else {
          color565 = ledmatrix.color565(0, 0, 0);
        }
        ledmatrix.drawPixel(x, y, color565);
      } else {
        //color565 = ledmatrix.color565( matrix[y][x],0,0);
        color565 = ledmatrix.color565( 10,50,50);
        ledmatrix.drawPixel(x, y, color565);
      }
    }
  }
  delay(2000);
}


void loopShow8x8Gradients(){
  uint8_t scale_factor = 18;
  for (int y=0; y<TOF_8x8_NUM_ROWS ; y++) {
    for (int x=0; x<TOF_8x8_NUM_COLS; x++) {
      uint16_t color565 = ledmatrix.color565(0, x*scale_factor,0);
      if(y<TOF_8x8_NUM_ROWS -1 ){
        ledmatrix.drawPixel(x, y, color565);
      } else {
        color565 = ledmatrix.color565( x*scale_factor,0,0);
        ledmatrix.drawPixel(x, y, color565);
      }
    }
  }
  delay(2000);
}

void loopShow8x8LastRow(){

  for (int y=0; y<TOF_8x8_NUM_ROWS ; y++) {
    for (int x=0; x<TOF_8x8_NUM_COLS; x++) {
      uint16_t color565 = ledmatrix.color565(0x00AA00);
      if(y<TOF_8x8_NUM_ROWS -1){
        ledmatrix.drawPixel(x, y, color565);
      } else {
        color565 = ledmatrix.color565(0xAA0000);
        ledmatrix.drawPixel(x, y, color565);
      }
    }
  }
  delay(2000);

}



void clearLEDMatrix(){

  ledmatrix.fill(0);

  // for (int y=0; y<ledmatrix.height(); y++) {
  //   for (int x=0; x<ledmatrix.width(); x++) {
  //     uint16_t color565 = ledmatrix.color565(0x000000);
  //     ledmatrix.drawPixel(x, y, color565);
  //   }
  // }
}

void loopShowLastRow(){

  for (int y=0; y<ledmatrix.height(); y++) {
    for (int x=0; x<ledmatrix.width(); x++) {
      uint16_t color565 = ledmatrix.color565(0x00AA00);
      if(y<ledmatrix.height() -1){
        ledmatrix.drawPixel(x, y, color565);
      } else {
        color565 = ledmatrix.color565(0xAA0000);
        ledmatrix.drawPixel(x, y, color565);
      }
    }
  }
  delay(2000);

}

void loopSameColor() {


  for (int y=0; y<ledmatrix.height(); y++) {
    for (int x=0; x<ledmatrix.width(); x++) {
      uint16_t color565 = ledmatrix.color565(0xAA0000);
      ledmatrix.drawPixel(x, y, color565);
    }
  }
  delay(2000);


  
  for (int y=0; y<ledmatrix.height(); y++) {
    for (int x=0; x<ledmatrix.width(); x++) {
      uint16_t color565 = ledmatrix.color565(0x00AA00);
      ledmatrix.drawPixel(x, y, color565);
    }
  }
  delay(2000);

  for (int y=0; y<ledmatrix.height(); y++) {
    for (int x=0; x<ledmatrix.width(); x++) {
      uint16_t color565 = ledmatrix.color565(0x0000AA);
      ledmatrix.drawPixel(x, y, color565);
    }
  }
  delay(2000);



}

void loopSwirlDemo(){
  uint32_t i = 0;
  for (int y=0; y<ledmatrix.height(); y++) {
    for (int x=0; x<ledmatrix.width(); x++) {
      uint32_t color888 = ledmatrix.ColorHSV(i * 65536 / 117 + hue_offset);
      uint16_t color565 = ledmatrix.color565(color888);
      ledmatrix.drawPixel(x, y, color565);
      i++;
    }
  }

  hue_offset += 256;

  ledmatrix.setGlobalCurrent(hue_offset / 256); // Demonstrate global current
}
