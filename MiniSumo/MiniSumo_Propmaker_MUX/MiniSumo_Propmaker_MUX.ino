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
bool lineDetectedArray[3];
#define LINE_NONE         -1
#define LINE_FRONT_LEFT   0
#define LINE_FRONT_RIGHT  1
#define LINE_REAR_CENTER  2





// VL53L1X TOF laser sensor
#define NUM_DISTANCE_SENSORS 3  // @TODO expand this to 3 for final design
SFEVL53L1X **distanceSensorArray; //Create pointer to a set of pointers to the sensor class
bool botDetectedArray[3];
#define BOT_NONE          -1
#define BOT_FRONT_LEFT    0
#define BOT_FRONT_CENTER  1
#define BOT_FRONT_RIGHT   2

#define MAX_BOT_DIST_CM 77
#define MAX_BOT_DIST_MM 770
#define MAX_BOT_DIST_IN 30




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
    
  Serial.println("start initLineSensors() ---------------");
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
  delay(1000);

  //Initialize all the distance sensors
  bool allSensorsSuccess = true;

  // @TODO start port number after last of the distance sensors
  int totalSensors = NUM_DISTANCE_SENSORS + NUM_LINE_SENSORS;
  for (byte port = 0 ; port < totalSensors; port++)
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
      Serial.print("Line Sensor: ");
      Serial.print(port);
      Serial.println(" configured");
    }
    delay(500);
  }

  if (allSensorsSuccess == false)
  {
    Serial.print("ERROR: Line Sensor initialization failed. exiting");
    // @TODO Error display here
    while(1)
    ;
  }

  Serial.println("Line sensors initialized.");  
}

// Setup both line and distance sensors using QWIIC MUX
void setupSensors() {

  initMUX();

  initDistanceSensors();

  //initLineSensors();

}


void initDistanceSensors(){

  Serial.println("Enter initDistanceSensors() -------------");
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
    Serial.print("set port:");
    Serial.print(port);
    int status  = distanceSensorArray[port]->begin(Wire);
    Serial.print(" status:");
    Serial.println(status);
    if (status == INIT_DISTANCE_SENSOR_FAIL)
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
    delay(500);
  }

  if (allSensorsSuccess == false)
  {
    Serial.print("ERROR: Sensor initialization failed. exiting");
    // @TODO Error display here
    while(1)
    ;
  }

  Serial.println("Exit initDistanceSensors. Distance sensors initialized. -------------");  
}

void loop()
{
  loopSensors();

  delay(180); //Wait for next reading
}

// 1. Handle line sensors
// 2. Handle bot sensors
void loopSensors(){

  // line sensors have to be first, before bot sensors
  //int lineStatus = loopLineSensors();
  // if(lineStatus != LINE_NONE){
  //   return;
  // }

  int botStatus = loopBotSensors();

}


    // if (tcs.available()) // if current measurement has done
    // {
    //     TCS34725::Color color = tcs.color();
    //     Serial.print("Color Temp : "); Serial.println(tcs.colorTemperature());
    //     Serial.print("Lux        : "); Serial.println(tcs.lux());
    //     Serial.print("R          : "); Serial.println(color.r);
    //     Serial.print("G          : "); Serial.println(color.g);
    //     Serial.print("B          : "); Serial.println(color.b);
    //     delay(3000);
    // }
int loopLineSensors(){
  int lineStatus = LINE_NONE;



  // Need this because the port numbering is:3-5, but whe what 0-2
  int sensorIndex = 0;
  TCS34725::Color color;

  // Loop thru the MUX ports for line sensors: 3-5
  byte start= NUM_DISTANCE_SENSORS;
  byte end= NUM_DISTANCE_SENSORS + NUM_LINE_SENSORS;
  for (byte port = start; port < end; port++)
  {
    myMux.setPort(port);                               //Tell mux to connect to this port, and this port only
    color = lineSensorArray[port]->color();

    if(isLineDetected(color)){
      lineDetectedArray[port] = true;
      Serial.print("line sensor port:");
      Serial.print(port);
      Serial.print(" detected:");
      Serial.println(lineDetectedArray[port]);
      }
  }

  Serial.println();  



  return lineStatus;
}

bool isLineDetected(TCS34725::Color color){
  bool isLineDetected = false;
  float r, g, b;

  // guessing at some values.
  // @TODO also look at lux and temperature to
  if(r > 100.0 && g > 100.0 && b > 100.0){
    isLineDetected = true;
  }

      // Serial.print("Color Temp : "); Serial.println(tcs.colorTemperature());
      // Serial.print("Lux        : "); Serial.println(tcs.lux());
      // Serial.print("R          : "); Serial.println(color.r);
      // Serial.print("G          : "); Serial.println(color.g);
      // Serial.print("B          : "); Serial.println(color.b);

  return isLineDetected;
}

int loopBotSensors(){
  int botStatus = BOT_NONE;
  int distance[NUM_DISTANCE_SENSORS];
  float distanceFeet;
  float distanceCM;
  float distanceMM;

  // Loop thru the MUX ports for bot/lidar sensors: 0-2
  for (byte port = 0; port < NUM_DISTANCE_SENSORS; port++)
  {
    myMux.setPort(port);                               //Tell mux to connect to this port, and this port only
    distance[port] = distanceSensorArray[port]->getDistance(); //Get the result of the measurement from the sensor

    distanceMM = distance[port];
    distanceCM = distanceMM /10.0;
    distanceFeet = (distance[port] * 0.0393701) / 12.0; // TODO calculate this from distanceMM


    // Serial.print("\tDistance");
    // Serial.print(sensorIndex);
    // Serial.print("(mm): ");
    // Serial.print(distanceMM);

    Serial.print("\tDistance");
    Serial.print(port);
    Serial.print("(cm): ");
    Serial.print(distanceCM, 2);
    if(distanceCM <  MAX_BOT_DIST_CM){
      botDetectedArray[port];
    }
  }

  Serial.println();  

  return botStatus;
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

