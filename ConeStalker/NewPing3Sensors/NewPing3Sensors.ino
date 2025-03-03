// ---------------------------------------------------------------------------
// Example NewPing library sketch that pings 3 sensors 20 times a second.
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
// Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(3, 3, MAX_DISTANCE) // 4, 5
  ,NewPing(5, 5, MAX_DISTANCE) // 6, 7
  //,NewPing(8, 9, MAX_DISTANCE)
};

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Serial.print(i);
    Serial.print("=");
    Serial.print(sonar[i].ping_in());
    Serial.print("in ");
  }
  Serial.println();
}