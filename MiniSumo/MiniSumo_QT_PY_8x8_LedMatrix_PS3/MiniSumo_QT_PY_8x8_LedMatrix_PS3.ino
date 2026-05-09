// Rainbow swirl example for the Adafruit IS31FL3741 13x9 PWM RGB LED
// Matrix Driver w/STEMMA QT / Qwiic connector. This is the simplest
// version and should fit on small microcontrollers like Arduino Uno.
// Tradeoff is that animation isn't always as smooth as seen in the
// buffered example. Each LED changes state immediately when accessed,
// there is no show() or display() function as with NeoPixels or some
// OLED screens.

#include <Adafruit_IS31FL3741.h>
#include <SparkFun_VL53L5CX_Library.h>  //http://librarymanager/All#SparkFun_VL53L5CX
#include "MedianFilterLib2.h"           // For getting an average value.

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

// Median Filter - to remove abnormal data readings.
// Define number of sensors and filter window size
//const int NUM_SENSORS = 3; //
const int WINDOW_SIZE = 5;

// Array of MedianFilter objects
MedianFilter2<int> filter[NUM_ROWS][NUM_COLS] =

{
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)}
};

long countdownMedianValues = 10 * 1000;
boolean medianValuesComplete = false;





// Some boards have just one I2C interface, but some have more...
TwoWire *i2c = &Wire1; // e.g. change this to &Wire1 for QT Py RP2040

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit QT RGB Matrix Simple RGB Swirl Test");

  setupLedMatrix();
  setup8x8();

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

  myImager.setResolution(NUM_ROWS*NUM_COLS); //Enable all 64 pads
  
  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  myImager.startRanging();

  Serial.print(F("Sparkfun TOF 8x8 Sensor initialized  with imageWidth:"));
  Serial.println(imageWidth);

  // Put this on display.
  //display.print("8x8");
  clearLEDMatrix();
  uint16_t  key_color = ledmatrix.color565(160, 32, 240); // purple
  uint16_t value_color = ledmatrix.color565(0, 150, 0);    // green
  int delay_time = 1500;

  ledMatrixKeyValueColor("8x", "OK", key_color, value_color, delay_time);


  delay(2000);
}


void setupLedMatrix(){

  if (! ledmatrix.begin(IS3741_ADDR_DEFAULT, i2c)) {
    Serial.println("IS41 not found");
    while (1);
  }

  Serial.println("IS41 found!");

  // By default the LED controller communicates over I2C at 400 KHz.
  // Arduino Uno can usually do 800 KHz, and 32-bit microcontrollers 1 MHz.
  i2c->setClock(800000);

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
  uint16_t  key_color = ledmatrix.color565(160, 32, 240); // purple
  uint16_t value_color = ledmatrix.color565(0, 150, 0);    // green
  int delay_time = 1500;

  ledMatrixKeyValueColor("LM", "OK", key_color, value_color, delay_time);

  clearLEDMatrix();

}

uint16_t hue_offset = 0;

void loop() {

  //loopSameColor();
  //loopShowLastRow();
  //loopShow8x8LastRow();
  //loopShow8x8Gradients();
  //loopReadFromMatrix();
  //loopSimulateMiniSumo();
  //loopMenu();
  //loopMenuColored();

  loop8x8Baseline(); // Take readings and average them out.
}


void loop8x8Baseline(){
  long time_slice = 100;


  if((abs(countdownMedianValues) < 80) && (medianValuesComplete == false)){
    medianValuesComplete = true;
    Serial.println("loop8x8Baseline() finished");
    int d;

    int sum, avg_r6, avg_r7;
    sum = 0;
    Serial.print("row7\t");
    for(int col=0 ; col<NUM_COLS ; col++){
      d = filter[6][col].GetFiltered();
      sum += d;
      Serial.print(d);
      Serial.print("\t");
    }
    Serial.println("");
    avg_r6 = sum/8;

    sum = 0;
    Serial.print("row8\t");
    for(int col=0 ; col<NUM_COLS ; col++){
      d = filter[7][col].GetFiltered();
      sum += d;
      Serial.print(d);
      Serial.print("\t");
    }
    Serial.println("");
    avg_r7 = sum/8;


  uint16_t key_color, value_color;

  key_color = ledmatrix.color565(160, 32, 240); // purple
  value_color = ledmatrix.color565(0, 150, 0);    // green

    String value;

    value = String(avg_r6);
    ledMatrixKeyValueColor("E6", value, key_color, value_color, 1000);
    value = String(avg_r7);
    ledMatrixKeyValueColor("E7", value, key_color, value_color, 1000);
    return;
  }

  if(medianValuesComplete){
    return;
  }
  
  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      int raw_d, filtered_d;
      int row, col;
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
          if(y > 0){
            row = y/8;
          }else {
            row = y;
          }
          Serial.print("countdown:"); Serial.print(countdownMedianValues);
          Serial.print("\trow:"); Serial.print(row);
        
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          if(true){
            raw_d = measurementData.distance_mm[x + y];

            // Save for later
            col = x;
            filtered_d = filter[row][col].GetFiltered();

            if(abs(raw_d - filtered_d) < 300){
              filter[row][col].AddValue(raw_d);
              dist8x8[row][col] = filtered_d;
            } else {
              //Serial.print("Anomoly!");
              //Serial.print("raw_d:");
              //Serial.println(raw_d);
            }
            Serial.print("\t");
            Serial.print(filtered_d);

          }
        }
        if(true){
          Serial.println();
        }
      }
      Serial.println();
    }
  }

  countdownMedianValues -= time_slice;
  delay(time_slice);
}


void loop8x8Raw(){
  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      int raw_d, filtered_d;
      int row, col;
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
          if(y > 0){
            row = y/8;
          }else {
            row = y;
          }
          Serial.print("row:"); Serial.print(row);
        
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          if(true){
            raw_d = measurementData.distance_mm[x + y];

            // Save for later
            col = x;
            filtered_d = filter[row][col].GetFiltered();

            if(abs(raw_d - filtered_d) < 300){
              filter[row][col].AddValue(raw_d);
              dist8x8[row][col] = filtered_d;
            } else {
              //Serial.print("Anomoly!");
              //Serial.print("raw_d:");
              //Serial.println(raw_d);
            }
            Serial.print("\t");
            Serial.print(filtered_d);

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

void loopMenuColored(){

  String strArray[] = {"E1", "E2", "OP"};

  uint16_t key_color, value_color;

  key_color = ledmatrix.color565(160, 32, 240); // purple
  value_color = ledmatrix.color565(0, 150, 0);    // green


  int delay_time = 1500;
  ledMatrixKeyValueColor(strArray[0], "1", key_color, value_color, delay_time);
  ledMatrixKeyValueColor(strArray[1], "4", key_color, value_color, delay_time);
  ledMatrixKeyValueColor(strArray[2], "7", key_color, value_color, delay_time);

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

void loopMenu(){

  String str = "FOO";
  String strArray[] = {"E1", "E2", "OP"};
  uint16_t color565;
  color565 = ledmatrix.color565(160, 32, 240); // purple
  ledmatrix.setTextColor(color565); // No background color needed


  
    int delay_time = 1500;
    ledMatrixKeyValue(strArray[0], "1", delay_time);
    ledMatrixKeyValue(strArray[1], "4", delay_time);
    ledMatrixKeyValue(strArray[2], "9", delay_time);
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
