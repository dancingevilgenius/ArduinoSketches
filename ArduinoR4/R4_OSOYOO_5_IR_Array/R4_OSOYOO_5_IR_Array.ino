const int irPin = A0; // the tracking module is attached to pin 2
const int ledPin = LED_BUILTIN;     // built-in LED is on pin 13

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

int irPins[SensorCount] = {A0, A1, A2, A3,A4};


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
  for(int i=0 ; i<SensorCount ;  i++){
    sensorValues[i] = digitalRead(irPins[i]);    
    Serial.print(sensorValues[i]); Serial.print(" ");
  }
  Serial.println();

  delay(2000);
  
}