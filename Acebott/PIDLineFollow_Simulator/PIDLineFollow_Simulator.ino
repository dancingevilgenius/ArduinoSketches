
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

int sensorPosition = SENSOR_POS_CENTER; // A single value used represent line relative to sensor array.


/**************************************************************************
  Sensor Array object initialisation 
*************************************************************************/
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int irPins[SensorCount] = {32, 33, 34, 35,36};

/*************************************************************************
  PID control system variables 
*************************************************************************/
float Kp = 0.07;  //related to the proportional control term; change the value by trial - and-error(ex : 0.07).
float Ki = 0.0008;  //related to the integral control term; change the value by trial-and-error (ex: 0.0008).
float Kd = 0.6;  //related to the derivative control term; change the value by trial - and-error(ex : 0.6).
int P;
int I;
int D;

/*************************************************************************
*
  Global variables
*************************************************************************/
int lastError = 0;
boolean onoff = false;

/*************************************************************************
* Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const uint8_t maxspeeda = 254;
const uint8_t maxspeedb = 254;
const uint8_t basespeeda = 150;
const uint8_t basespeedb = 150;





int position;
int center_line = 3500;

void setup() {
  Serial.begin(9600);

  // digital pins to simulate 5 line sensors
  for(int i=0 ; i<SensorCount ; i++){
    pinMode(irPins[i], INPUT);  
  }
 
}

void loop() {
  /*
  position = random(0, 7000);

  Serial.print("position:");
  Serial.print(position);
  Serial.print(",");
  Serial.print("Variable_2:");
  Serial.println(center_line);
  delay(100);
  */
  PIDControl();
}


void PIDControl() {

  uint16_t position = getSensorArrayPosition(); // Returns value between 0-7000
  int error = SENSOR_POS_CENTER - position;       // 3500 is the ideal position (the centre)

  P = error;
  I = I + error;
  D = error - lastError;

  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;  //calculate the correction needed to be applied to the speed


  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;


  // Make sure values are within min and max bounds
  motorspeeda = getMotorSpeedWithinBounds(motorspeeda);
  motorspeedb = getMotorSpeedWithinBounds(motorspeedb);


  Serial.print("P/E:");Serial.print(error);
  Serial.print(" I:");Serial.print(I);
  Serial.print(" D:");Serial.print(D);
  Serial.print(" spdA:");Serial.print(motorspeeda);
  Serial.print(" spdB:");Serial.print(motorspeedb);
  Serial.println();


  motorController(motorspeeda, motorspeedb);
}

void motorController(uint16_t motorSpeedA, uint16_t motorSpeedB){
  // Stub to enable compile. Do nothing
}


int getMotorSpeedWithinBounds(int speed){
  int newSpeed = speed;
  if (speed > maxspeedb) {
      newSpeed = maxspeedb;
  }
  else if (speed < 0) {
    newSpeed = maxspeeda/5;
  }

  return newSpeed;
}





int getSensorArrayPosition(){
  boolean triggerOnWhite = false;
  return getOsoyooSensorPosition(triggerOnWhite);
}

int getOsoyooSensorPosition(boolean triggerOnWhite){

  //const uint8_t SensorCount = 5;
  uint16_t sensorValues[SensorCount]; // Store sensor array boolean states in here.


  // Get the raw binary values from the 5 sensor array
  int numSensorHits=0;
  for(int i=0 ; i<SensorCount ;  i++){
    // Thite line on black background
    if(triggerOnWhite){
      sensorValues[i] = digitalRead(irPins[i]);
    } else {
      // Flip values for black lines on white background
      sensorValues[i] = !digitalRead(irPins[i]);
    }
    if(sensorValues[i]){
      numSensorHits++;  
    }        
  }

  boolean printDebug = false;
  int val = getOsoyooPositionByArrayValues(numSensorHits, sensorValues, printDebug);


  return val;
}

int getOsoyooPositionByArrayValues(int numSensorHits, uint16_t sensorValues[], boolean printDebug){
    if(numSensorHits == 0){
      // No sensors, real bad
      if(printDebug){
        Serial.println("zero sensor hits. Bad if we are on a track racing.");
      }
    } else if(numSensorHits == 1){
      if(sensorValues[SENSOR_OUTER_LEFT]){
        sensorPosition = SENSOR_POS_16P;
        if(printDebug){
          Serial.print("B1 Outer left:"); Serial.println(sensorPosition, DEC);
        }        
      } else if(sensorValues[SENSOR_INNER_LEFT]){
        sensorPosition = SENSOR_POS_32P;
        if(printDebug) {
          Serial.print("B1 Inner left:");Serial.println(sensorPosition, DEC);
        }        
      } else if(sensorValues[SENSOR_CENTER]){
        sensorPosition = SENSOR_POS_CENTER;
        if(printDebug){
          Serial.print("B1 Center:");Serial.println(sensorPosition, DEC);
        }        
      } else if(sensorValues[SENSOR_INNER_RIGHT]){
        sensorPosition = SENSOR_POS_66P;
        if(printDebug){
          Serial.print("B1 Inner right:");Serial.println(sensorPosition, DEC);
        }
        
      } else if(sensorValues[SENSOR_OUTER_RIGHT]){
        sensorPosition = SENSOR_POS_83P;
        if(printDebug)
        {
          Serial.print("B1 Outer right:");Serial.println(sensorPosition, DEC);        
        }
      }
  } else if(numSensorHits == 2){
    if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT]){
      sensorPosition = SENSOR_POS_32P;
      if(printDebug) {
        Serial.print("B2 Hard left:");Serial.println(sensorPosition, DEC);
      }
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_POS_16P;
      if(printDebug) {
        Serial.print("B2 Hard left:");Serial.println(sensorPosition, DEC);
      }
      
    } else if(sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_POS_MIN;
      if(printDebug) {
        Serial.print("B2 Hard left:");Serial.println(sensorPosition, DEC);
      }
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      if(printDebug) {
        Serial.print("B2 Hard right:");Serial.println(sensorPosition, DEC);
      }
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      if(printDebug) {
        Serial.print("B2 Hard right:");Serial.println(sensorPosition, DEC);
      }
    } else if(sensorValues[SENSOR_INNER_RIGHT] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      if(printDebug) {
        Serial.print("B2 Hard right:");Serial.println(sensorPosition, DEC);      
      }
    }
  } else if(numSensorHits == 3){
    // intersection or end zone
    sensorPosition = SENSOR_POS_CENTER;
    if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_INNER_RIGHT]){
      sensorPosition = SENSOR_POS_CENTER;
      if(printDebug) {
        Serial.print("B3+ Perpendicular line or end solid shape:");Serial.println(sensorPosition, DEC);      
      }
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_POS_MIN;
      if(printDebug) {
        Serial.print("B3 Hard left:"); Serial.println(sensorPosition, DEC);
      }
    }  else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_RIGHT] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      if(printDebug) {
        Serial.print("B3 Hard right:");Serial.println(sensorPosition, DEC);
      }
    } else {
      sensorPosition = SENSOR_POS_CENTER;
      if(printDebug) {
        Serial.print("B3 Perpendicular line or end solid shape:"); Serial.println(sensorPosition, DEC);
      }
    }
  } else if(numSensorHits == 4){
      sensorPosition = SENSOR_POS_CENTER;
      if(printDebug) {
        Serial.print("B4 Perpendicular line or end solid shape:"); Serial.println(sensorPosition, DEC);
      }
  } else if(numSensorHits == 5){
      sensorPosition = SENSOR_POS_CENTER;
      if(printDebug) {
        Serial.print("B5 Perpendicular line or end solid shape:"); Serial.println(sensorPosition, DEC);
      }
  }

  return sensorPosition;
}


