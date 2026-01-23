
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





const int ledPin = LED_BUILTIN;


int sensorPosition = SENSOR_POS_CENTER; // A single value used represent line relative to sensor array.

// Vars for IR/Color sensor array
const uint8_t SensorCount = 5;
int irPins[SensorCount] = {A4, A3, A2, A1,A0};
//uint16_t sensorValues[SensorCount]; // Store sensor array boolean states in here.


void setup() {
  Serial.begin(115200);

  Serial.println("Entering setup() -------------");
  setupOsoyooSensorArray();


  pinMode(ledPin, OUTPUT);     // set ledPin as OUTPUT
  Serial.println("Exiting setup() --------------");
}


void setupOsoyooSensorArray(){
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
}


void loop() {

  bool triggerOnWhite = false;
  bool printRawValues = true;
  sensorPosition = getOsoyooSensorPosition(triggerOnWhite, printRawValues);
  //handleMotors(sensorPosition);
  delay(250);
  
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


int getOsoyooSensorPosition(boolean triggerOnWhite, bool printValues){

  uint16_t sensorValues[SensorCount]; // Store sensor array boolean states in here.


  // Get the raw binary values from the 5 sensor array
  int numSensorHits=0;
  if(printValues){
  Serial.print("Sensors");
  }

  for(int i=0 ; i<SensorCount ;  i++){
    if(triggerOnWhite){
      sensorValues[i] = digitalRead(irPins[i]);
    } else {
      // Flip values
      sensorValues[i] = !digitalRead(irPins[i]);
      numSensorHits++;  
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

    // Serial.print("numSensorHits:");
    // Serial.println(numSensorHits);
    if(true){
      return -1;
    }

    if(numSensorHits == 0){
      // No sensors, real bad
      Serial.println("zero sensor hits. Bad if we are on a track racing.");
    } else if(numSensorHits == 1){
      if(sensorValues[SENSOR_OUTER_LEFT]){
        sensorPosition = SENSOR_POS_16P;
        Serial.print("B1 Outer left:"); Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_INNER_LEFT]){
        sensorPosition = SENSOR_POS_32P;
        Serial.print("B1 Inner left:");Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_CENTER]){
        sensorPosition = SENSOR_POS_CENTER;
        Serial.print("B1 Center:");Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_INNER_RIGHT]){
        sensorPosition = SENSOR_POS_66P;
        Serial.print("B1 Inner right:");Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_OUTER_RIGHT]){
        sensorPosition = SENSOR_POS_83P;
        Serial.print("B1 Outer right:");Serial.println(sensorPosition, DEC);        
      }
  } else if(numSensorHits == 2){
    if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT]){
      sensorPosition = SENSOR_POS_32P;
      Serial.print("B2 Hard left:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_POS_16P;
      Serial.print("B2 Hard left:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_POS_MIN;
      Serial.print("B2 Hard left:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      Serial.print("B2 Hard right:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      Serial.print("B2 Hard right:");Serial.println(sensorPosition, DEC);
    } else if(sensorValues[SENSOR_INNER_RIGHT] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      Serial.print("B2 Hard right:");Serial.println(sensorPosition, DEC);      
    }
  } else if(numSensorHits == 3){
    // intersection or end zone
    sensorPosition = SENSOR_POS_CENTER;
    if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_INNER_RIGHT]){
      sensorPosition = SENSOR_POS_CENTER;
      Serial.print("B3+ Perpendicular line or end solid shape:");Serial.println(sensorPosition, DEC);      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_POS_MIN;
      Serial.print("B3 Hard left:"); Serial.println(sensorPosition, DEC);
    }  else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_RIGHT] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      Serial.print("B3 Hard right:");Serial.println(sensorPosition, DEC);
    } else {
      sensorPosition = SENSOR_POS_CENTER;
      Serial.print("B3 Perpendicular line or end solid shape:"); Serial.println(sensorPosition, DEC);
    }
  } else if(numSensorHits == 4){
      sensorPosition = SENSOR_POS_CENTER;
      Serial.print("B4 Perpendicular line or end solid shape:"); Serial.println(sensorPosition, DEC);
  } else if(numSensorHits == 5){
      sensorPosition = SENSOR_POS_CENTER;
      Serial.print("B5 Perpendicular line or end solid shape:"); Serial.println(sensorPosition, DEC);
  }
}







