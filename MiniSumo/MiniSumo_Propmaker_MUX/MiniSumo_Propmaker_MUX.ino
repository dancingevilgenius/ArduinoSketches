/*
  Sumo/Mini Sumo project that uses QWIIC MUX to handle multiple line and multiple distance sensors.
  Use the Qwiic Mux to access multiple I2C devices on seperate busses.
*/

#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include <TCS34725.h>


// Sparkfun QWIIC multiplexor
QWIICMUX myMux;



// RBG line sensors
#define NUM_LINE_SENSORS 1        // @TODO expand this to 3 for final design
//TCS34725 tcs;
TCS34725 **lineSensorArray;     //Create pointer to a set of pointers to the sensor class






// VL53L1X TOF laser sensor
#define NUM_DISTANCE_SENSORS 1  // @TODO expand this to 3 for final design
SFEVL53L1X **distanceSensorArray; //Create pointer to a set of pointers to the sensor class

// Defines for sensor initialization.
// Yes, they are opposite of each other.
#define INIT_DISTANCE_SENSOR_SUCCESS 0
#define INIT_DISTANCE_SENSOR_FAIL 1
#define INIT_LINE_SENSOR_SUCCESS 1
#define INIT_LINE_SENSOR_FAIL 0

void setup()
{
  Serial.begin(115200);
  Serial.println("Qwiic Mux Shield Read Example");
  delay(2000);


  Wire.begin();

  setupSensors();
}

void initLineSensors(){
    
    // if (!tcs.attach(Wire))
    //     Serial.println("ERROR: TCS34725 NOT FOUND !!!");

    // tcs.integrationTime(33); // ms
    // tcs.gain(TCS34725::Gain::X01);

  //Create set of pointers to the class
  lineSensorArray = new TCS34725 *[NUM_LINE_SENSORS];

  //Assign pointers to instances of the class
  for (int x = 0; x < NUM_LINE_SENSORS; x++){
    lineSensorArray[x] = new TCS34725();
    lineSensorArray[x]->attach(Wire);
    lineSensorArray[x]->integrationTime(33); // ms   @TODO change this to match the other sensor array of 180ms
    lineSensorArray[x]->gain(TCS34725::Gain::X01);
  }



  // Not sure if the next 3 lines are necessary for anything other than sanity check.
  byte currentPortNumber = myMux.getPort();
  Serial.print("CurrentPort: ");
  Serial.println(currentPortNumber);

  //Initialize all the distance sensors
  bool allSensorsSuccess = true;

  // @TODO start port number after last of the distance sensors
  int totalSensors = NUM_DISTANCE_SENSORS + NUM_LINE_SENSORS;
  for (byte port = NUM_DISTANCE_SENSORS ; port < totalSensors; port++)
  {
    myMux.setPort(port);
    if (lineSensorArray[port]->attach(Wire) == INIT_LINE_SENSOR_FAIL)
    {
      Serial.print("ERROR: Line Sensor ");
      Serial.print(port);
      Serial.println(" did not begin! Check wiring");
      allSensorsSuccess = false;
    }
    else
    {
      lineSensorArray[port]->integrationTime(33); // ms   @TODO change this to match the other sensor array of 180ms
      lineSensorArray[port]->gain(TCS34725::Gain::X01);

      //Configure each sensor
      Serial.print("Line Sensor ");
      Serial.print(port);
      Serial.println(" configured");
    }
  }

  if (allSensorsSuccess == false)
  {
    Serial.print("ERROR: Line Sensor initialization failed. exiting");
    // @TODO Error display here
    while(1)
    ;
  }

  Serial.println("Distance sensors initialized.");  



}

// Setup both line and distance sensors using QWIIC MUX
void setupSensors() {

  initMUX();

  initDistanceSensors();

  initLineSensors();

}


void initDistanceSensors(){
  //Create set of pointers to the class
  distanceSensorArray = new SFEVL53L1X *[NUM_DISTANCE_SENSORS];

  //Assign pointers to instances of the class
  for (int x = 0; x < NUM_DISTANCE_SENSORS; x++){
    distanceSensorArray[x] = new SFEVL53L1X(Wire);
  }


  // Not sure if the next 3 lines are necessary for anything other than sanity check.
  byte currentPortNumber = myMux.getPort();
  Serial.print("CurrentPort: ");
  Serial.println(currentPortNumber);

  //Initialize all the distance sensors
  bool allSensorsSuccess = true;

  for (byte port = 0; port < NUM_DISTANCE_SENSORS; port++)
  {
    myMux.setPort(port);
    if (distanceSensorArray[port]->begin(Wire) == INIT_DISTANCE_SENSOR_FAIL)
    {
      Serial.print("ERROR: Sensor ");
      Serial.print(port);
      Serial.println(" did not begin! Check wiring");
      allSensorsSuccess = false;
    }
    else
    {
      //Configure each sensor
      distanceSensorArray[port]->setIntermeasurementPeriod(180);
      distanceSensorArray[port]->setDistanceModeLong();
      distanceSensorArray[port]->startRanging(); //Write configuration bytes to initiate measurement
      Serial.print("Sensor ");
      Serial.print(port);
      Serial.println(" configured");
    }
  }

  if (allSensorsSuccess == false)
  {
    Serial.print("ERROR: Sensor initialization failed. exiting");
    // @TODO Error display here
    while(1)
    ;
  }

  Serial.println("Distance sensors initialized.");  
}

void loop()
{
  loopSensors();

  delay(180); //Wait for next reading
}


void loopSensors(){
  int distance[NUM_DISTANCE_SENSORS];
  float distanceFeet;

  for (byte x = 0; x < NUM_DISTANCE_SENSORS; x++)
  {
    myMux.setPort(x);                               //Tell mux to connect to this port, and this port only
    distance[x] = distanceSensorArray[x]->getDistance(); //Get the result of the measurement from the sensor

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

}


void initMUX(){
  // Initialize MUX
  if (myMux.begin(QWIIC_MUX_DEFAULT_ADDRESS, Wire) == false)
  {
    Serial.println("ERROR: Mux not detected. Freeze");
    // @TODO Error display here    
    while (1)
      ;
  }
  Serial.println("Mux detected");
}

