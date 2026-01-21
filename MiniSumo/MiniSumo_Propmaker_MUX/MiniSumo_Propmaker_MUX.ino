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


// User/Boot button
#define BUILTIN_LED 13    // Normally defined on other Arduino boards.
const int buttonPin = PIN_BUTTON;  // the number of the pushbutton pin
const int ledPin = BUILTIN_LED;    // the number of the LED pin. 13
int buttonState = 0;  // variable for reading the pushbutton status



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

// For timer
#define TIME_SLICE 180
long timeElapsed = 0;
bool firstButtonPress = false;
time_t timeOfFirstButtonPress = -1;
bool fightStarted = false;




void loop()
{

  loopBuiltInButton();

  loopFightCheck();

  loopSensors();

  delay(TIME_SLICE); //Wait for next reading
}

bool loopFightCheck(){

    if(fightStarted){
      return true;
    }

    if(firstButtonPress){

      time_t dt = time(nullptr) - timeOfFirstButtonPress;
      Serial.println(dt);

      if(dt >= 5 && !fightStarted){
        fightStarted = true;
        Serial.print("It's time! at ");
        Serial.print(dt);
        Serial.println(" seconds since start/boot button press");
        return true;
      }
    }

  return false;
}

void loopBuiltInButton(){
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  } else {
    float timeElapsedFloat = float(timeElapsed)/ 1000.0;
    // turn LED off:
    digitalWrite(ledPin, LOW);
    //Serial.print("timeElapsed:");
    //Serial.println(timeElapsedFloat, 1);

    if(!firstButtonPress){
      firstButtonPress = true;
      timeElapsed = 0; // timer start
      
      timeOfFirstButtonPress = time(nullptr);
      Serial.print("First button press at:");
      Serial.println(timeOfFirstButtonPress);      
    }  else {
      time_t dt = time(nullptr) - timeOfFirstButtonPress;
      Serial.print("time since 1st button press:");
      Serial.println(dt);
    }

  }

  timeElapsed += TIME_SLICE;
  
}


void setup()
{
  Serial.begin(115200);
  Serial.println("Qwiic Mux Shield Read Example");
  delay(2000);

  Wire.begin();

  setupBuiltInButton();


  setupSensors();
}


void setupBuiltInButton(){
  Serial.println("setupBuiltInButton()");
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
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
    
    Serial.print(" status");
    Serial.print(port);
    Serial.print("\t");
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
      distanceSensorArray[port]->setDistanceModeShort(); // Carlos changed this from Long to Short
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


// 1. Handle line sensors
// 2. Handle bot sensors
void loopSensors(){

  // line sensors have to be first, before bot sensors
  //int lineStatus = loopLineSensors();
  // if(lineStatus != LINE_NONE){
  //   return;
  // }

  int botStatus = loopBotSensors(false);

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

int loopBotSensors(bool print){
  int botStatus = BOT_NONE;
  int distance[NUM_DISTANCE_SENSORS];
  float distanceFeet;
  float distanceCM;
  float distanceMM;
  int status;

  // Loop thru the MUX ports for bot/lidar sensors: 0-2
  for (byte port = 0; port < NUM_DISTANCE_SENSORS; port++)
  {
    myMux.setPort(port);                               //Tell mux to connect to this port, and this port only
    distance[port] = distanceSensorArray[port]->getDistance(); //Get the result of the measurement from the sensor

    distanceMM = distance[port];
    distanceCM = distanceMM /10.0;
    distanceFeet = (distance[port] * 0.0393701) / 12.0; // TODO calculate this from distanceMM

    // 1. If the distance is 5cm or less, this is basically out of normal range response
    // 2. We do not want to track anything further than the width of the ring (roughly 30" / 77cm)
    if((distanceCM > 5.0) && (distanceCM <  MAX_BOT_DIST_CM)){
      botDetectedArray[port] = true;
    } else {
      botDetectedArray[port] = false;
    }


    if(true){
      if(print){
        Serial.print("\tdetected");
        Serial.print(port);
        Serial.print(":");
        Serial.print(botDetectedArray[port]);
      }
    } else {
      if(print){
        Serial.print("\tDistance");
        Serial.print(port);
        Serial.print("(cm): ");
        Serial.print(distanceCM, 2);
      }
    }

  }

  if(print){
    Serial.println();
  }

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

