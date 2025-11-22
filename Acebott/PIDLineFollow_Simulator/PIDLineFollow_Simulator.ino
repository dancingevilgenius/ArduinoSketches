
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
  position = random(0, 7000);

  Serial.print("position:");
  Serial.print(position);
  Serial.print(",");
  Serial.print("Variable_2:");
  Serial.println(center_line);
  delay(100);
}

int getPosition(){
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


