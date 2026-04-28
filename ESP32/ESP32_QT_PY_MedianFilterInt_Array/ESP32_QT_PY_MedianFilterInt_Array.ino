#include "MedianFilterLib2.h"

// Define number of sensors and filter window size
const int NUM_SENSORS = 3;
const int WINDOW_SIZE = 5;

// Array of MedianFilter objects
MedianFilter2<int> filter[NUM_SENSORS][1] = 
{
  {MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE)}
};

void setup() {
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Simulate reading sensors
    int rawValue = analogRead(A0 + i); 
    
    // Add value to corresponding filter
    filter[i][0].AddValue(rawValue);
    
    // Get filtered value
    int filteredValue = filter[i][0].GetFiltered();
    
    Serial.print("Sensor "); Serial.print(i);
    Serial.print(" Raw: "); Serial.print(rawValue);
    Serial.print(" Median: "); Serial.println(filteredValue);
  }
  delay(100);
}
