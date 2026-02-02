#include <Wire.h>
#include <Adafruit_TCS34725.h> // For RGB line sensor. More accurate than IR sensors. Possibly slower


// Sensor Array for RGB/TCS34725 defined below
#define SENSOR_OUTER_LEFT   0 // Not used
#define SENSOR_INNER_LEFT   1
#define SENSOR_CENTER       2
#define SENSOR_INNER_RIGHT  3
#define SENSOR_OUTER_RIGHT  4 // Not used


// Convert sensor values to a relative position of line.
// Arbitrary scale from left to right of 0-7000, center 3500
// For drag racing, we should only need the middle 3 sensors
#define SENSOR_POS_MIN 0
#define SENSOR_POS_CENTER 3500
#define SENSOR_POS_MAX 7000

// If the sensors are evenly spaced, give them scaled values 0-7000
#define SENSOR_POS_00P    1166  // Hard left
#define SENSOR_POS_16P    1166  // Hard left
#define SENSOR_POS_32P    2331  // Slight left
#define SENSOR_POS_50P    3500  // Center
#define SENSOR_POS_66P    4277  // Slight left
#define SENSOR_POS_83P    5833  // Hard right
#define SENSOR_POS_100P   7000
#define SENSOR_POS_ERROR -1

#ifdef ESP32
  // ESP32 board is missing these defines
  // Yes the pins are not sequential
  #define A0 33 // 14
  #define A1 25 // 27
  #define A2 26 // 26
  #define A3 27 // 25
  #define A4 14 //33
  #define LED_BUILTIN 2
#endif

#ifndef A0
  // ESP32 board is missing these defines
  // Yes the pins are not sequential
  #define A0 5 // 14
  #define A1 6 // 27
  #define A2 7 // 26
  #define A3 8 // 25
  #define A4 9 //33
#endif

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif


const int ledPin = LED_BUILTIN;


int sensorPosition = SENSOR_POS_CENTER; // A single value used represent line relative to sensor array.


/* Initialise with specific int time and gain values */
#define SENSOR_COUNT 3
Adafruit_TCS34725 tcs[SENSOR_COUNT]; // Initialize in setup()
const int tcsPins[SENSOR_COUNT] = { 4,5,6};
volatile boolean isRGBReady[SENSOR_COUNT] = {false, false, false};
bool sensorValues[SENSOR_COUNT]; // line/no-line black/white


void setup() {
  Serial.begin(115200);

  Serial.println("Entering setup() -------------");

  pinMode(ledPin, OUTPUT);     // set ledPin as OUTPUT

  // TCS34725 Color Sensors
  setupRGBSensors();

  Serial.println("Exiting setup() --------------");
  delay(4000);
}



void setupRGBSensors() {

  tcs[1] = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
  tcs[2] = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

  for(int i=0 ; i<SENSOR_COUNT ; i++){

    tcs[i] = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

    pinMode(tcsPins[i], INPUT_PULLUP); //TCS interrupt output is Active-LOW and Open-Drain

    if(i == 0){
      attachInterrupt(digitalPinToInterrupt(tcsPins[i]), isrLeft, FALLING);
    }
    else if(i == 1){
      attachInterrupt(digitalPinToInterrupt(tcsPins[i]), isrCenter, FALLING);
    }
    else if(i == 2){
      attachInterrupt(digitalPinToInterrupt(tcsPins[i]), isrRight, FALLING);
    }

    Serial.print(i); Serial.print(": RBG sensor");
    if (tcs[i].begin()) {
      Serial.print("found."); 
    } else {
      Serial.println("not found ... check your connections. exiting program.");
      exit(2);
    }
  
    // Set persistence filter to generate an interrupt for every RGB Cycle, regardless of the integration limits
    tcs[i].write8(TCS34725_PERS, TCS34725_PERS_NONE); 
    tcs[i].setInterrupt(true);
  }
  
  Serial.flush();
  Serial.println("setupRGBSensors() complete.");
  delay(1000);
}


/* tcs.getRawData() does a delay(Integration_Time) after the sensor readout.
We don't need to wait for the next integration cycle because we receive an interrupt when the integration cycle is complete*/
void getRawData_noDelay(int index, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs[index].read16(TCS34725_CDATAL);
  *r = tcs[index].read16(TCS34725_RDATAL);
  *g = tcs[index].read16(TCS34725_GDATAL);
  *b = tcs[index].read16(TCS34725_BDATAL);
}




void loop() {

  bool triggerOnWhite = false;
  bool printRawValues = false;
  //handleMotors(sensorPosition);
  loopRGBPrintValues();

}

void loopRGBPrintValues(){

  uint16_t r, g, b, c, colorTemp, lux;

  for(int i=0 ; i<SENSOR_COUNT ; i++){
    if (isRGBReady[i]) {
      getRawData_noDelay(i, &r, &g, &b, &c);
      colorTemp = tcs[i].calculateColorTemperature(r, g, b);
      lux = tcs[i].calculateLux(r, g, b);
      
      Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
      Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
      Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
      Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
      Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
      Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
      Serial.println(" ");
      Serial.flush();

      // Calculate black line hits here
      sensorValues[i] = isRGBLineHit(i, r, g, b);

      
      tcs[i].clearInterrupt();
      isRGBReady[i] = false;
    }
  }

}

// Determine algorithmically if we are on black or white.
// Will need to get some test data.
// Placeholder algorithm.
bool isRGBLineHit(int index, uint16_t r, uint16_t g, uint16_t b){

  if(r <100 && g<100 && b<100){
    return true;
  }

  return false;

}





int calculatePositionForOne(int index){

  int sensorPosition = SENSOR_POS_ERROR;

    if(index == SENSOR_OUTER_LEFT){
      sensorPosition = SENSOR_POS_16P;
      Serial.print(sensorPosition, DEC);
      Serial.println("\tB1 Outer left:"); 
      
    } else if(index == SENSOR_INNER_LEFT){
      sensorPosition = SENSOR_POS_32P;
      Serial.print(sensorPosition, DEC);
      Serial.println("\tB1 Inner left:");
      
    } else if(index == SENSOR_CENTER){
      sensorPosition = SENSOR_POS_CENTER;
      Serial.print(sensorPosition, DEC);
      Serial.println("\tB1 Center:");
      
    } else if(index == SENSOR_INNER_RIGHT){
      sensorPosition = SENSOR_POS_66P;
      Serial.print(sensorPosition, DEC);
      Serial.println("\tB1 Inner right:");
      
    } else if(index == SENSOR_OUTER_RIGHT){
      sensorPosition = SENSOR_POS_83P;
      Serial.print(sensorPosition, DEC);        
      Serial.println("\tB1 Outer right:");
    }

  return SENSOR_POS_ERROR;
}

int getPositionByArrayValues(int numSensorHits, uint16_t sensorValues[], bool printValues){

  int position = SENSOR_POS_ERROR;

  if(printValues){
    Serial.print("numSensorHits:");                                                                                       
    Serial.print(numSensorHits);
  }

  if(numSensorHits == 1){
    for(int i=0 ; i<SENSOR_COUNT ; i++){
        if(printValues){
          Serial.print("\t");
          Serial.print(sensorValues[i]);
        }
        if(sensorValues[i]){
          position = calculatePositionForOne(i);
          Serial.println();

          return position;
        }
    }
  }

  return SENSOR_POS_ERROR;

}



// This is just a placeholder. The PID implementation will be defined in a different project
void handleMotors(int position){
    Serial.println("handleMotors() TODO not implemented");

    // Put in a dead zone for center
    int diff = abs(position - SENSOR_POS_CENTER);
    if(diff < 200 ){
      // close enough to center. do nothing
      return;
    }
}


//Interrupt Service Routine
void isrLeft() 
{
  isRGBReady[0] = true;
}

//Interrupt Service Routine
void isrCenter() 
{
  isRGBReady[1] = true;
}

//Interrupt Service Routine
void isrRight() 
{
  isRGBReady[2] = true;
}




