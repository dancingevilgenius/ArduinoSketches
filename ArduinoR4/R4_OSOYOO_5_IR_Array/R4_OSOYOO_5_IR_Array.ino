const int irPin = A0; // the tracking module is attached to pin 2
const int ledPin = LED_BUILTIN;     // built-in LED is on pin 13

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

#define SENSOR_OUTER_LEFT   1
#define SENSOR_INNER_LEFT   0
#define SENSOR_CENTER       2
#define SENSOR_INNER_RIGHT  3
#define SENSOR_OUTER_RIGHT   4

int irPins[SensorCount] = {A4, A3, A2, A1,A0};


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
    Serial.print("sum:");
    Serial.println(sum);
    if(sum == 0){
      // No sensors, real bad
    } else if(sum == 1){
      if(sensorValues[SENSOR_OUTER_LEFT]){
        Serial.println("Outer left");
      } else if(sensorValues[SENSOR_INNER_LEFT]){
        Serial.println("Inner left");
      } else if(sensorValues[SENSOR_CENTER]){
        Serial.println("Center");
      } else if(sensorValues[SENSOR_INNER_RIGHT]){
        Serial.println("Inner right");
      } else if(sensorValues[SENSOR_OUTER_RIGHT]){
        Serial.println("Outer right");
      }
  }

  delay(2000);
  
}