const int irPin = A0; // the tracking module is attached to pin 2
const int ledPin = LED_BUILTIN;     // built-in LED is on pin 13

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

#define SENSOR_OUTER_LEFT   1
#define SENSOR_INNER_LEFT   0
#define SENSOR_CENTER       2
#define SENSOR_INNER_RIGHT  3
#define SENSOR_OUTER_RIGHT   4

#define SENSOR_ARRAY_MIN 0
#define SENSOR_ARRAY_CENTER 3500
#define SENSOR_ARRAY_MAX 7000

#define SENSOR_ARRAY_16P    1166
#define SENSOR_ARRAY_32P    2331
#define SENSOR_ARRAY_50P    3500
#define SENSOR_ARRAY_66P    4277
#define SENSOR_ARRAY_83P    5833
#define SENSOR_ARRAY_100P   7000


int irPins[SensorCount] = {A4, A3, A2, A1,A0};

int sensorPosition = SENSOR_ARRAY_CENTER;

void setup() {
  Serial.begin(115200);

  //pinMode(irPin, INPUT); // set trackingPin as INPUT

  pinMode(A0, INPUT); // set trackingPin as INPUT
  pinMode(A1, INPUT); // set trackingPin as INPUT
  pinMode(A2, INPUT); // set trackingPin as INPUT
  pinMode(A3, INPUT); // set trackingPin as INPUT
  pinMode(A4, INPUT); // set trackingPin as INPUT

  pinMode(ledPin, OUTPUT);     // set ledPin as OUTPUT
}

void loop() {
  /*
  boolean val = digitalRead(irPin); // read the value from the tracking module

  if (val == HIGH) { // if the sensor reads HIGH (white surface)
    digitalWrite(ledPin, LOW); // turn off the LED
    Serial.println("Detect: White!");
  } else { // if the sensor reads LOW (black line)
    digitalWrite(ledPin, HIGH); // turn on the LED
    Serial.println("Detect: Black!");
  }  
  delay(100); // a small delay for readability
  */
  int sum=0;
  for(int i=0 ; i<SensorCount ;  i++){
    sensorValues[i] = digitalRead(irPins[i]);    
    sum += sensorValues[i];
    //Serial.print(sensorValues[i]); Serial.print(" ");
  }
    //Serial.print("sum:");
    //Serial.println(sum);
    if(sum == 0){
      // No sensors, real bad
    } else if(sum == 1){
      if(sensorValues[SENSOR_OUTER_LEFT]){
        sensorPosition = SENSOR_ARRAY_16P;
        Serial.print("1 Outer left:"); Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_INNER_LEFT]){
        sensorPosition = SENSOR_ARRAY_32P;
        Serial.print("1 Inner left:");Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_CENTER]){
        sensorPosition = SENSOR_ARRAY_CENTER;
        Serial.print("1 Center:");Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_INNER_RIGHT]){
        sensorPosition = SENSOR_ARRAY_66P;
        Serial.print("1 Inner right:");Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_OUTER_RIGHT]){
        sensorPosition = SENSOR_ARRAY_83P;
        Serial.print("1 Outer right:");Serial.println(sensorPosition, DEC);        
      }
  } else if(sum == 2){
    if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT]){
      sensorPosition = SENSOR_ARRAY_32P;
      Serial.print("2 Hard left:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_ARRAY_16P;
      Serial.print("2 Hard left:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_ARRAY_MIN;
      Serial.print("2 Hard left:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_RIGHT]){
      sensorPosition = SENSOR_ARRAY_MAX;
      Serial.print("2 Hard right:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_ARRAY_MAX;
      Serial.print("2 Hard right:");Serial.println(sensorPosition, DEC);
    } else if(sensorValues[SENSOR_INNER_RIGHT] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_ARRAY_MAX;
      Serial.print("2 Hard right:");Serial.println(sensorPosition, DEC);      
    }
  } else if(sum >= 3){
    // intersection or end zone
    sensorPosition = SENSOR_ARRAY_CENTER;
    if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_INNER_RIGHT]){
      sensorPosition = SENSOR_ARRAY_CENTER;
      Serial.print("3+ Perpendicular line or end solid shape:");Serial.println(sensorPosition, DEC);      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_ARRAY_MIN;
      Serial.print("3 Hard left:"); Serial.println(sensorPosition, DEC);
    }  else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_RIGHT] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_ARRAY_MAX;
      Serial.print("3 Hard right:");Serial.println(sensorPosition, DEC);
    } else {
      sensorPosition = SENSOR_ARRAY_CENTER;
      Serial.println("3+ Perpendicular line or end solid shape:"); Serial.println(sensorPosition, DEC);
    }
    
  }


  delay(2000);
  
}