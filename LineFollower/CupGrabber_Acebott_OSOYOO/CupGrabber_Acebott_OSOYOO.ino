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
  #define A0 5 // 14
  #define A1 16 // 27
  #define A2 17 // 26
  #define A3 18 // 25
  #define A4 19 //33
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
#define TIME_SLICE 100   // Tenth of a second
#define ONE_SECOND 1000
#define AUTO_TIMEOUT 1500 * 1000 // Set this for really long for testing.
#define IGNORE_START_LINE_TIMEOUT 1500
#define CROSSED_FINISH_LINE_TIMEOUT 1500 * 1000

bool isRaceStarted = false;
long raceStartTimeMS = -1;
long crossedFinishLineTimeMS = -1;


// Define the control inputs
#define MOT_A1_PIN 12   // og 10
#define MOT_A2_PIN 13   // og 9
#define MOT_B1_PIN 25    // og 6
#define MOT_B2_PIN 26    // og 5
#define SLP_PIN 13
#define MOTOR_MAX_POWER 255  // Full range is 255 (fwd) to -255 (rev)
int powerMotorA = 0;
int powerMotorB = 0;




const int ledPin = LED_BUILTIN;


int sensorPosition = SENSOR_POS_CENTER; // A single value used represent line relative to sensor array.

// Vars for IR/Color sensor array
#define SENSOR_COUNT 5
int irPins[SENSOR_COUNT] = {A0, A1, A2, A3, A4};
//uint16_t sensorValues[SensorCount]; // Store sensor array boolean states in here.


void setup() {
  Serial.begin(115200);

  Serial.println("Entering setup() -------------");
  setupOsoyooSensorArray();


  pinMode(ledPin, OUTPUT);     // set ledPin as OUTPUT


  // For start/stop buttons
  setupPS3Controller();

  setupMotors();

  Serial.println("Exiting setup() --------------");
}



void stopMotors(){
  // Turn off motors - Initial state
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

}


/// Set the current on a motor channel using PWM and directional logic.
///
/// \param pwm    PWM duty cycle ranging from -255 full reverse to 255 full forward
/// \param IN1_PIN  pin number xIN1 for the given channel
/// \param IN2_PIN  pin number xIN2 for the given channel
void setMotor(int pwm, int IN1_PIN, int IN2_PIN)
{
  // Range checking of input power.
  if(pwm > 255){
    pwm = MOTOR_MAX_POWER;
  } else if(pwm < -255){
    pwm = -MOTOR_MAX_POWER;
  }


  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

/// Set the current on both motors.
///
/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255
void setMotors(int pwm_A, int pwm_B)
{  
  setMotor(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  setMotor(pwm_B, MOT_B1_PIN, MOT_B2_PIN);

  // Print a status message to the console.
  Serial.print("Set motor A PWM = ");
  Serial.print(pwm_A);
  Serial.print(" motor B PWM = ");
  Serial.println(pwm_B);
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

    isRaceStarted = true;

}



void loop() {

  bool triggerOnWhite = false;
  bool printRawValues = true;

  if(isRaceStarted == false){
    delay(TIME_SLICE);
    return;
  }

  handleEndRaceConditions();


  //Serial.println("raceStarted!");
  //Serial.println(isRaceStarted);

  sensorPosition = getOsoyooSensorPosition(triggerOnWhite, printRawValues);
  //handleMotors(sensorPosition);


  delay(TIME_SLICE);  
}


void handleEndRaceConditions(){

  long currentTime = millis();

  long dt = currentTime - raceStartTimeMS;

  // Auto timeout 
  if(dt > AUTO_TIMEOUT){
    isRaceStarted = false;
    Serial.print("Bot stopped automatically after 10 seconds");
    stopMotors();
  }


  // Stop motors a short amount of time after finish line crossed
  if(crossedFinishLineTimeMS > 0) {
    dt = currentTime - crossedFinishLineTimeMS;
    if(dt > CROSSED_FINISH_LINE_TIMEOUT){
      isRaceStarted = false;
      Serial.print("Bot stopped a little bit after crossing finish line ");
      Serial.print(dt);
      Serial.println(" milliseconds after crossing finish line.");
      stopMotors();
    }
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

int getPositionFromIndex(int index, bool printValues){

  int sensorPosition = -1;

    if(index == SENSOR_OUTER_LEFT){
      sensorPosition = SENSOR_POS_16P;
      if(printValues) {
        Serial.print("B1 Outer left:"); Serial.println(sensorPosition, DEC);
      }
      
    } else if(index == SENSOR_INNER_LEFT){
      sensorPosition = SENSOR_POS_32P;
      if(printValues) {
        Serial.print("B1 Inner left:");Serial.println(sensorPosition, DEC);
      }
      
    } else if(index == SENSOR_CENTER){
      sensorPosition = SENSOR_POS_CENTER;
      if(printValues) {
        Serial.print("B1 Center:");Serial.println(sensorPosition, DEC);
      }
      
    } else if(index == SENSOR_INNER_RIGHT){
      sensorPosition = SENSOR_POS_66P;
      if(printValues) {
        Serial.print("B1 Inner right:");Serial.println(sensorPosition, DEC);
      }
      
    } else if(index == SENSOR_OUTER_RIGHT){
      sensorPosition = SENSOR_POS_83P;
      if(printValues) {
        Serial.print("B1 Outer right:");Serial.println(sensorPosition, DEC);        
      }
    }

  return sensorPosition;
}

// Returns sensor position in range 0-7000
int getOsoyooPositionByArrayValues(int numSensorHits, uint16_t sensorValues[]){

    //Serial.print("numSensorHits:");
    //Serial.print(numSensorHits);
    // if(true){
    //   return -1;
    // }


  // Initialize to invalid
  int sensorPosition = -1;

  if(numSensorHits == 0){
    // No sensors, real bad
    Serial.println("Maybe in between sensors. Keep going at last known motor speeds");
  }

  if(numSensorHits == 1){

    Serial.print("hits:");
    Serial.print(numSensorHits);
    bool printValues = false;

    for(int i=0 ; i<SENSOR_COUNT ; i++){
      Serial.print("\t");
      Serial.print(sensorValues[i]);        
      if(sensorValues[i]){
        sensorPosition = getPositionFromIndex(i, false);
      }
    }
    Serial.print("\tpos:");
    Serial.print(sensorPosition);
    Serial.println();
  }


  // Time since race started
  long dt = millis() - raceStartTimeMS;
  bool isTestingForFinishLine = dt > IGNORE_START_LINE_TIMEOUT;

  if(isTestingForFinishLine){
    if(numSensorHits > 2){
      if(crossedFinishLineTimeMS < 0){
        Serial.print("Finish line detected. Motors will stop shortly after ");
        Serial.print(CROSSED_FINISH_LINE_TIMEOUT);
        Serial.println(" milliseconds.");
        crossedFinishLineTimeMS = millis();
      }
    }
  }


  return sensorPosition;


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

// For drag race, we might be able to get away without using PID
void handleMotors(int position){
  
    Serial.println("handleMotors()");

    // Put in a dead zone for center
    int diff = abs(position - SENSOR_POS_CENTER);
    if(diff < 200 ){
      // close enough to center. do nothing
      powerMotorA = MOTOR_MAX_POWER;
      powerMotorB = MOTOR_MAX_POWER;
      
    } else {

      diff = position - SENSOR_POS_CENTER;
      
      if(diff < 0){ // We are to the left
        powerMotorA = MOTOR_MAX_POWER;
        powerMotorB = MOTOR_MAX_POWER * 0.8;
      } else {      // We are to the right
        powerMotorA = MOTOR_MAX_POWER * 0.8;
        powerMotorB = MOTOR_MAX_POWER;
      }
    }


    setMotors(powerMotorA, powerMotorB);
}



void setupMotors(void)
{
  // Set all the motor control inputs to OUTPUT
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);

  pinMode(SLP_PIN, OUTPUT);

  // Turn off motors - Initial state
  stopMotors();

  digitalWrite(SLP_PIN, HIGH);

  Serial.println("Exiting setupMotors()");

}



