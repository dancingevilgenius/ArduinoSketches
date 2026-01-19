/*
  Sumo/Mini Sumo project that uses QWIIC MUX to handle multiple line and multiple distance sensors.
  Use the Qwiic Mux to access multiple I2C devices on seperate busses.
*/

#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#define NUM_DISTANCE_SENSORS 1



QWIICMUX myMux;
SFEVL53L1X **distanceSensor; //Create pointer to a set of pointers to the sensor class

#define INIT_SENSOR_SUCCESS 0
#define INIT_SENSOR_FAIL 1

void setup()
{
  Serial.begin(115200);
  Serial.println("Qwiic Mux Shield Read Example");
  delay(2000);


  Wire.begin();

  setupSensors();


}

// Setup both line and distance sensors using QWIIC MUX
void setupSensors() {
  //Create set of pointers to the class
  distanceSensor = new SFEVL53L1X *[NUM_DISTANCE_SENSORS];

  //Assign pointers to instances of the class
  for (int x = 0; x < NUM_DISTANCE_SENSORS; x++)
    distanceSensor[x] = new SFEVL53L1X(Wire);

  if (myMux.begin(QWIIC_MUX_DEFAULT_ADDRESS, Wire) == false)
  {
    Serial.println("Mux not detected. Freezing...");
    while (1)
      ;
  }
  Serial.println("Mux detected");

  byte currentPortNumber = myMux.getPort();
  Serial.print("CurrentPort: ");
  Serial.println(currentPortNumber);

  //Initialize all the sensors
  bool initSuccess = true;

  for (byte x = 0; x < NUM_DISTANCE_SENSORS; x++)
  {
    myMux.setPort(x);
    if (distanceSensor[x]->begin(Wire) == INIT_SENSOR_FAIL)
    {
      Serial.print("Sensor ");
      Serial.print(x);
      Serial.println(" did not begin! Check wiring");
      initSuccess = false;
    }
    else
    {
      //Configure each sensor
      distanceSensor[x]->setIntermeasurementPeriod(180);
      distanceSensor[x]->setDistanceModeLong();
      distanceSensor[x]->startRanging(); //Write configuration bytes to initiate measurement
      Serial.print("Sensor ");
      Serial.print(x);
      Serial.println(" configured");
    }
  }

  if (initSuccess == false)
  {
    Serial.print("Sensor initialization failed. exiting");
    exit(1);
  }

  Serial.println("Mux Shield online");
}

void loop()
{
  int distance[NUM_DISTANCE_SENSORS];
  float distanceFeet;

  for (byte x = 0; x < NUM_DISTANCE_SENSORS; x++)
  {
    myMux.setPort(x);                               //Tell mux to connect to this port, and this port only
    distance[x] = distanceSensor[x]->getDistance(); //Get the result of the measurement from the sensor

    Serial.print("\tDistance");
    Serial.print(x);
    Serial.print("(mm): ");
    Serial.print(distance[x]);

    distanceFeet = (distance[x] * 0.0393701) / 12.0;

    Serial.print("\tDistance");
    Serial.print(x);
    Serial.print("(ft): ");
    Serial.print(distanceFeet, 2);
  }

  Serial.println();

  delay(180); //Wait for next reading
}