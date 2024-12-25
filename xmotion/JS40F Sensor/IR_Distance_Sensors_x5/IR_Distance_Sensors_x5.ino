
#include <xmotion.h>
#define NUM_DISTANCE_SENSORS 5
int distance_pins[NUM_DISTANCE_SENSORS]=  {A1,A2, A4, A5, 4};
int distance_values[NUM_DISTANCE_SENSORS]; //We declared some variable for storing ML1 output

void setup() {
  
  for(int i=0 ; i < NUM_DISTANCE_SENSORS ; i++){
    pinMode(distance_pins[i], INPUT);
  }
  
  Serial.begin(9600); // Serial communication started with 9600 bits per second.
}

void loop() {
  
  for(int i=0 ;  i < NUM_DISTANCE_SENSORS ; i++){
    distance_values[i]=digitalRead(distance_pins[i]); 
    Serial.print(distance_pins[i]);
    Serial.print(":"); //It will write this words to serial monitor
    Serial.print(distance_values[i]); //Controller will send serial monitor to raw value.
    Serial.print(" ");
  }
  
  Serial.println("");
  delay(100);  // we are adding 100 ms delay for delay at readings. 
  }

