// Author: Carlos Garcia
// December 2026

#include <Ps3Controller.h> // Needs the "Fork of PS3 Controller Host" not the "PS3 Controller Host" library


// Sensor Array defined below
#define SENSOR_OUTER_LEFT   1
#define SENSOR_INNER_LEFT   0
#define SENSOR_CENTER       2
#define SENSOR_INNER_RIGHT  3
#define SENSOR_OUTER_RIGHT   4


// Convert sensor values to a relative position of line.
// Arbitrary scale from left to right of 0-7000, center 3500
#define SENSOR_POS_MIN 0
#define SENSOR_POS_CENTER 3500
#define SENSOR_POS_MAX 7000
#define SENSOR_POS_16P    1166
#define SENSOR_POS_32P    2331
#define SENSOR_POS_50P    3500
#define SENSOR_POS_66P    4277
#define SENSOR_POS_83P    5833
#define SENSOR_POS_100P   7000

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
  #define A0 33 // 14
  #define A1 25 // 27
  #define A2 26 // 26
  #define A3 27 // 25
  #define A4 14 //33
#endif

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif


#define PS3_BLACK_BLACK_1   "00:19:c1:c2:d8:01"
#define PS3_BLACK_BLACK_2   "00:19:c1:c2:d8:02" 
#define PS3_BLACK_BLACK_3   "00:19:c1:c2:d8:03" 
#define PS3_BLUE_BLACK_1    "00:19:c1:c2:ee:01"

// PS3 Controller
int player = 0;
int battery = 0;


// Race timer
#define TIME_SLICE 250   // Quarter of a second
#define ONE_SECOND 1000
#define AUTO_TIMEOUT 10000
bool isRaceStarted = false;
long raceStartTimeMS = -1;


// Define the control inputs
#define MOT_A1_PIN 2   // og 10
#define MOT_A2_PIN 4   // og 9
#define MOT_B1_PIN 18    // og 6
#define MOT_B2_PIN 19    // og 5

#define SLP_PIN 13





const int ledPin = LED_BUILTIN;


int sensorPosition = SENSOR_POS_CENTER; // A single value used represent line relative to sensor array.

// Vars for IR/Color sensor array
#define SENSOR_COUNT 5
int irPins[SENSOR_COUNT] = {A4, A3, A2, A1, A0};
//uint16_t sensorValues[SensorCount]; // Store sensor array boolean states in here.


void setup() {
  Serial.begin(115200);

  Serial.println("Entering setup() -------------");
  setupOsoyooSensorArray();


  pinMode(ledPin, OUTPUT);     // set ledPin as OUTPUT


  // For start/stop buttons
  setupPS3Controller();


  Serial.println("Exiting setup() --------------");
}


void setupOsoyooSensorArray(){
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
}

void setupPS3Controller(){

    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);

    Ps3.begin(PS3_BLACK_BLACK_1);

    Ps3.setPlayer(player);

    //-------------------- Player LEDs -------------------
    Serial.print("Setting LEDs to player "); Serial.println(player, DEC);

    Serial.println("Press the 'P3' logo to bind the PS3 controller. Should see 4 LEDS flash");
    Serial.println("Exiting setupPS3Controller()");

}



void loop() {

  bool triggerOnWhite = false;
  bool printRawValues = false;

  if(isRaceStarted == false){
    delay(100);
    return;
  }

  //handleTimeout();


  //Serial.println("raceStarted!");
  //Serial.println(isRaceStarted);

  sensorPosition = getOsoyooSensorPosition(triggerOnWhite, printRawValues);
  //handleMotors(sensorPosition);


  delay(TIME_SLICE);  
}


void handleTimeout(){

  long dt = millis() - raceStartTimeMS;

  if(dt > AUTO_TIMEOUT){
    isRaceStarted = false;
    Serial.print("Bot stopped automatically after 10 seconds");
  }
}




int getOsoyooSensorPosition(boolean triggerOnWhite, bool printValues){

  uint16_t sensorValues[SENSOR_COUNT]; // Store sensor array boolean states in here.


  // Get the raw binary values from the 5 sensor array
  int numSensorHits=0;
  if(printValues){
  Serial.print("Sensors");
  }

  for(int i=0 ; i<SENSOR_COUNT ;  i++){
    if(triggerOnWhite){
      sensorValues[i] = digitalRead(irPins[i]);
    } else {
      // Flip values
      sensorValues[i] = !digitalRead(irPins[i]);
    }
    if(sensorValues[i]){
      numSensorHits++;
    }
    if(printValues){
      Serial.print("\t");  
      Serial.print(sensorValues[i]);  
    }

  }
  if(printValues){
    Serial.println();
  }



  sensorPosition = getOsoyooPositionByArrayValues(numSensorHits, sensorValues);


  return sensorPosition;    
}


int getOsoyooPositionByArrayValues(int numSensorHits, uint16_t sensorValues[]){

    //Serial.print("numSensorHits:");
    //Serial.print(numSensorHits);
    // if(true){
    //   return -1;
    // }

  if(numSensorHits == 1){

    Serial.print("hits:");
    Serial.print(numSensorHits);

    for(int i=0 ; i<SENSOR_COUNT ; i++){
      Serial.print("\t");
      Serial.print(sensorValues[i]);
        
    }
    Serial.println();
  }

  return SENSOR_POS_CENTER;


  //   if(numSensorHits == 0){
  //     // No sensors, real bad
  //     Serial.println("zero sensor hits. Bad if we are on a track racing.");
  //   } else if(numSensorHits == 1){
  //     if(sensorValues[SENSOR_OUTER_LEFT]){
  //       sensorPosition = SENSOR_POS_16P;
  //       Serial.print("B1 Outer left:"); Serial.println(sensorPosition, DEC);
        
  //     } else if(sensorValues[SENSOR_INNER_LEFT]){
  //       sensorPosition = SENSOR_POS_32P;
  //       Serial.print("B1 Inner left:");Serial.println(sensorPosition, DEC);
        
  //     } else if(sensorValues[SENSOR_CENTER]){
  //       sensorPosition = SENSOR_POS_CENTER;
  //       Serial.print("B1 Center:");Serial.println(sensorPosition, DEC);
        
  //     } else if(sensorValues[SENSOR_INNER_RIGHT]){
  //       sensorPosition = SENSOR_POS_66P;
  //       Serial.print("B1 Inner right:");Serial.println(sensorPosition, DEC);
        
  //     } else if(sensorValues[SENSOR_OUTER_RIGHT]){
  //       sensorPosition = SENSOR_POS_83P;
  //       Serial.print("B1 Outer right:");Serial.println(sensorPosition, DEC);        
  //     }
  // } else if(numSensorHits == 2){
  //   if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT]){
  //     sensorPosition = SENSOR_POS_32P;
  //     Serial.print("B2 Hard left:");Serial.println(sensorPosition, DEC);
      
  //   } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_OUTER_LEFT]){
  //     sensorPosition = SENSOR_POS_16P;
  //     Serial.print("B2 Hard left:");Serial.println(sensorPosition, DEC);
      
  //   } else if(sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_OUTER_LEFT]){
  //     sensorPosition = SENSOR_POS_MIN;
  //     Serial.print("B2 Hard left:");Serial.println(sensorPosition, DEC);
      
  //   } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_RIGHT]){
  //     sensorPosition = SENSOR_POS_MAX;
  //     Serial.print("B2 Hard right:");Serial.println(sensorPosition, DEC);
      
  //   } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_OUTER_RIGHT]){
  //     sensorPosition = SENSOR_POS_MAX;
  //     Serial.print("B2 Hard right:");Serial.println(sensorPosition, DEC);
  //   } else if(sensorValues[SENSOR_INNER_RIGHT] && sensorValues[SENSOR_OUTER_RIGHT]){
  //     sensorPosition = SENSOR_POS_MAX;
  //     Serial.print("B2 Hard right:");Serial.println(sensorPosition, DEC);      
  //   }
  // } else if(numSensorHits == 3){
  //   // intersection or end zone
  //   sensorPosition = SENSOR_POS_CENTER;
  //   if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_INNER_RIGHT]){
  //     sensorPosition = SENSOR_POS_CENTER;
  //     Serial.print("B3+ Perpendicular line or end solid shape:");Serial.println(sensorPosition, DEC);      
  //   } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_OUTER_LEFT]){
  //     sensorPosition = SENSOR_POS_MIN;
  //     Serial.print("B3 Hard left:"); Serial.println(sensorPosition, DEC);
  //   }  else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_RIGHT] && sensorValues[SENSOR_OUTER_RIGHT]){
  //     sensorPosition = SENSOR_POS_MAX;
  //     Serial.print("B3 Hard right:");Serial.println(sensorPosition, DEC);
  //   } else {
  //     sensorPosition = SENSOR_POS_CENTER;
  //     Serial.print("B3 Perpendicular line or end solid shape:"); Serial.println(sensorPosition, DEC);
  //   }
  // } else if(numSensorHits == 4){
  //     sensorPosition = SENSOR_POS_CENTER;
  //     Serial.print("B4 Perpendicular line or end solid shape:"); Serial.println(sensorPosition, DEC);
  // } else if(numSensorHits == 5){
  //     sensorPosition = SENSOR_POS_CENTER;
  //     Serial.print("B5 Perpendicular line or end solid shape:"); Serial.println(sensorPosition, DEC);
  // }
}


void notify()
{
    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.event.button_down.cross ) {
        Serial.println("Started pressing the cross button");
    }
    if( Ps3.event.button_up.cross ){
        Serial.println("Released the cross button");
    }

    if( Ps3.event.button_down.square ){
        Serial.println("Started pressing the square button");
    }
    if( Ps3.event.button_up.square ) {
        Serial.println("Released the square button");
    }
    if( Ps3.event.button_down.triangle ) {
        Serial.println("Started pressing the triangle button");
    }
    if( Ps3.event.button_up.triangle ) {
        Serial.println("Released the triangle button");
    }

    if( Ps3.event.button_down.circle ) {
        Serial.println("Started pressing the circle button");
    }
    if( Ps3.event.button_up.circle ) {
        Serial.println("Released the circle button");
    }

    //--------------- Digital D-pad button events --------------
    if( Ps3.event.button_down.up ) {
        Serial.println("Started pressing the up button");
    }
    if( Ps3.event.button_up.up ) {
        Serial.println("Released the up button");
    }

    if( Ps3.event.button_down.right ) {
        Serial.println("Started pressing the right button");
    }
    if( Ps3.event.button_up.right ) {
        Serial.println("Released the right button");
    }

    if( Ps3.event.button_down.down ) {
        Serial.println("Started pressing the down button");
    }
    if( Ps3.event.button_up.down ) {
        Serial.println("Released the down button");
    }

    if( Ps3.event.button_down.left ) {
        Serial.println("Started pressing the left button");
    }
    if( Ps3.event.button_up.left ) {
        Serial.println("Released the left button");
    }

    //------------- Digital shoulder button events -------------
    if( Ps3.event.button_down.l1 ) {
        //Serial.println("Started pressing the left shoulder button");
    }
    if( Ps3.event.button_up.l1 ) {
        //Serial.println("Released the left shoulder button");
    }

    if( Ps3.event.button_down.r1 ) {
        //Serial.println("Started pressing the right shoulder button");
    }
    if( Ps3.event.button_up.r1 ) {
        //Serial.println("Released the right shoulder button");
    }

    //-------------- Digital trigger button events -------------
    if( Ps3.event.button_down.l2 ) {
        //Serial.println("Started pressing the left trigger button");
        Serial.println("Stop! - Race Stopped");
        isRaceStarted = false;
    }
    if( Ps3.event.button_up.l2 ) {
        //Serial.println("Released the left trigger button");
    }

    if( Ps3.event.button_down.r2 ) {
        //Serial.println("Started pressing the right trigger button");
        Serial.println("Go! - Race Started");
        isRaceStarted = true;
        raceStartTimeMS = millis();
    }
    if( Ps3.event.button_up.r2 ) {
        //Serial.println("Released the right trigger button");
    }

    //--------------- Digital stick button events --------------
    if( Ps3.event.button_down.l3 ) {
        Serial.println("Started pressing the left stick button");
    }
    if( Ps3.event.button_up.l3 ) {
        Serial.println("Released the left stick button");
    }

    if( Ps3.event.button_down.r3 ) {
        Serial.println("Started pressing the right stick button");
    }
    if( Ps3.event.button_up.r3 ) {
        Serial.println("Released the right stick button");
    }

    //---------- Digital select/start/ps button events ---------
    if( Ps3.event.button_down.select ) {
        Serial.println("Started pressing the select button");
    }
    if( Ps3.event.button_up.select ) {
        Serial.println("Released the select button");
    }

    if( Ps3.event.button_down.start ) {
        Serial.println("Started pressing the start button");
    }
    if( Ps3.event.button_up.start ) {
        Serial.println("Released the start button");
    }
    if( Ps3.event.button_down.ps ) {
        Serial.println("Started pressing the Playstation button");
    }
    if( Ps3.event.button_up.ps ) {
        Serial.println("Released the Playstation button");
    }

   //printJoystickRawValues();
   //handleJoystickChanges();



   //---------------------- Battery events ---------------------
    //printBatteryStatus();

}



void onConnect(){
    Serial.println("Connected to PS3");
    Serial.println("Press either right trigger to start race.");
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






