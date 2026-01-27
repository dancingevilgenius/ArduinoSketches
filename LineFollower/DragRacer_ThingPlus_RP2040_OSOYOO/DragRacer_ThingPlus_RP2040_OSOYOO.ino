
// Sensor Array defined below
#define SENSOR_OUTER_LEFT   0
#define SENSOR_INNER_LEFT   1
#define SENSOR_CENTER       2
#define SENSOR_INNER_RIGHT  3
#define SENSOR_OUTER_RIGHT  4


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

// Vars for IR/Color sensor array
#define SENSOR_COUNT 1
//int irPins[SENSOR_COUNT] = {A4, A3, A2, A1, A0};
int irPins[SENSOR_COUNT] = {A0};
//uint16_t sensorValues[SensorCount]; // Store sensor array boolean states in here.


void setup() {
  Serial.begin(115200);

  Serial.println("Entering setup() -------------");
  setupOsoyooSensorArray();


  pinMode(ledPin, OUTPUT);     // set ledPin as OUTPUT

  Serial.println("Exiting setup() --------------");
  delay(4000);

}


void setupOsoyooSensorArray(){
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  Serial.println("Exiting setupOsoyooSensorArray() ");
  delay(2000);
}


void loop() {
  Serial.println("loop 2");

  bool triggerOnWhite = false;
  bool printRawValues = false;
  //sensorPosition = getOsoyooSensorPosition(triggerOnWhite, printRawValues);
  //handleMotors(sensorPosition);
  delay(2000);
  
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



  sensorPosition = getOsoyooPositionByArrayValues(numSensorHits, sensorValues, false);


  return sensorPosition;    
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

int getOsoyooPositionByArrayValues(int numSensorHits, uint16_t sensorValues[], bool printValues){

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





