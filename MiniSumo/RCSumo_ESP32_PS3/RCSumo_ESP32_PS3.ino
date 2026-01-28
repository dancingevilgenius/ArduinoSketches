/*
  Sumo/Mini Sumo project that uses QWIIC MUX to handle multiple line and multiple distance sensors.
  Use the Qwiic Mux to access multiple I2C devices on seperate busses.
*/

#include <Wire.h>                             // For I2C/QWIIC/STEMMA devices
#include <SparkFun_I2C_Mux_Arduino_Library.h> // For Multiplexor to handle multiple devices with same address
#include "SparkFun_VL53L1X.h"                 // For distance/bot sensor
#include <FS_MX1508.h>                        // For DRV8871 motor driver
#include <Adafruit_NeoPixel.h>                // For built-in neopixel.  Or optional neopixel array
#include <QTRSensors.h>                       // Infrared reflectance/line sensor

#define ONE_SECOND 1000

// Built-in Neopixel
//#define PIN_NEOPIXEL 8 - Already defined in Feather support code
#define NUMPIXELS 1 // The board has one built-in NeoPixel
// Initialize the NeoPixel strip object
Adafruit_NeoPixel pixel(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
int neopixelCountdownTimer = 3 * ONE_SECOND;

// Motor controllers. One for each side
#define PIN_MOTOR_L_A 18 // Left Motor
#define PIN_MOTOR_L_B 19 // Left Motor
#define PIN_MOTOR_R_A 18 // Right Motor
#define PIN_MOTOR_R_B 19 // Right Motor
#define MAX_MOTOR_SPEED 100
MX1508 motorL(PIN_MOTOR_L_A, PIN_MOTOR_L_B); // default SLOW_DECAY (resolution 8 bits, frequency 1000Hz)
MX1508 motorR(PIN_MOTOR_R_A, PIN_MOTOR_R_B); // default SLOW_DECAY (resolution 8 bits, frequency 1000Hz)
#define REVERSE_TIME_MS 3000
int lineReverseCountdown = 3000;

// Sparkfun QWIIC multiplexor
QWIICMUX myMux;


// User/Boot button
#define BUILTIN_LED 13    // Normally defined on other Arduino boards.
const int buttonPin = PIN_BUTTON;  // the number of the pushbutton pin
const int ledPin = BUILTIN_LED;    // the number of the LED pin. 13
int buttonState = 0;  // variable for reading the pushbutton status




#define NUM_LINE_SENSORS 1        // @TODO expand this to 3 for final design
bool lineDetectedArray[NUM_LINE_SENSORS];
#define LINE_NONE         -1
#define LINE_FRONT_LEFT   0
#define LINE_FRONT_RIGHT  1
#define LINE_REAR_CENTER  2
QTRSensors qtr;
uint16_t sensorValues[NUM_LINE_SENSORS];




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
#define TIME_SLICE 100
long timeElapsed = 0;
bool firstButtonPress = false;
long timeOfFirstButtonPress = -1;
bool fightStarted = false;


void loop()
{

  loopBuiltInButton();

  fightStarted = loopFightCheck();

  loopSensors(fightStarted);

  //loopMotors(fightStarted);

  loopBuiltInNeopixel(fightStarted);

  delay(TIME_SLICE); //Wait for next reading
}



void loopBuiltInNeopixel(bool fightStarted) {

  if(!fightStarted){
    return;
  }  

  if(neopixelCountdownTimer <= 0){
    return;
  }

  neopixelCountdownTimer = neopixelCountdownTimer - TIME_SLICE;

  if(neopixelCountdownTimer > 0){
    Serial.print("loopBuiltInNeopixel() countdown:");
    Serial.println(neopixelCountdownTimer);
    if(abs(neopixelCountdownTimer % 1000) < 300){
      pixel.setPixelColor(0, pixel.Color(255, 255, 0));  
    } else {
      pixel.setPixelColor(0, pixel.Color(0, 0, 0));
    }
    pixel.show(); // Show the color

  } else {
    Serial.print("loopBuiltInNeopixel() countdown finished.");
    if(neopixelCountdownTimer < 0){
      neopixelCountdownTimer = 0;                
    }

    pixel.setPixelColor(0, pixel.Color(0, 255, 0));
    pixel.show();


  }
}


void loopNeopixelDemo(){
  // Set the color to Red (R, G, B)
  pixel.setPixelColor(0, pixel.Color(255, 0, 0));
  pixel.show(); // Show the color
  delay(500); // Wait 500ms

  // Turn off
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();
  delay(500); // Wait 500ms

  // Set the color to Green (R, G, B)
  pixel.setPixelColor(0, pixel.Color(0, 255, 0));
  pixel.show(); // Show the color
  delay(500); // Wait 500ms

  // Turn off
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();
  delay(500); // Wait 500ms


  // Set the color to Blue (R, G, B)
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
  pixel.show(); // Show the color
  delay(500); // Wait 500ms

  // Turn off
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();
  delay(500); // Wait 500ms
}

// @TODO check the sensors for what to do.
// right now, just running MX1508 library example simple_motor.
void loopMotors(bool fightStarted){
  if(fightStarted == false){
    return;
  }


  //loopMotorDemo();


}

void loopDemo2(){
  int duration=2500;
  driveMotors(100, 100);
  delay(duration);

  driveMotors(-100, -100);
  delay(duration);

  driveMotors(100, -100);
  delay(duration);
 
  driveMotors(-100, 100);
  delay(duration);
}

void driveMotors(int pctL, int pctR){

  if(pctL > MAX_MOTOR_SPEED){
    pctL = MAX_MOTOR_SPEED;
  }
  else if(pctL < -MAX_MOTOR_SPEED){
    pctL = -MAX_MOTOR_SPEED;
  }
  if(pctR > MAX_MOTOR_SPEED){
    pctR = MAX_MOTOR_SPEED;
  }
  else if(pctR < -MAX_MOTOR_SPEED){
    pctR = -MAX_MOTOR_SPEED;
  }

  motorL.motorGoP(pctL);
  motorR.motorGoP(pctR);

}

void loopMotorDemo(){
  Serial.println("Ramp up forward 0 to  MAX_MOTOR_SPEED");
  for (int pct = 0; pct <= 100; pct++) { // ramp up forward.
    motorL.motorGoP(pct);
    delay(50);
  }

  Serial.println("Motor stop: speed decrease slowly (free wheeling)");
  motorL.motorStop();  // free wheeling. Motor stops slowly
  delay(5 * ONE_SECOND);

  Serial.println("Ramp up backward 0 to  -MAX_MOTOR_SPEED");
  for (int pct = 0; pct <= 100; pct++) { // ramp up backward.
    motorL.motorGoP(-pct);
    delay(50);
  }

  Serial.println("Motor brake: speed decrease quickly ");
  motorL.motorBrake();  //  Fast , strong brake
  delay(5 * ONE_SECOND);
}



bool loopFightCheck(){

    if(fightStarted){
      return true;
    }

    if(firstButtonPress){

      long dt = millis() - timeOfFirstButtonPress;
      Serial.println(dt);

      if(dt >= 5*ONE_SECOND && !fightStarted){
        fightStarted = true;
        Serial.print("It's time!");
        Serial.print(dt);
        Serial.println(" seconds since start/boot button press.");
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

    long currentTimeMs = millis();
    if(!firstButtonPress){      
      firstButtonPress = true;
      timeElapsed = 0; // timer start
      
      timeOfFirstButtonPress = currentTimeMs;
      Serial.print("First button press at:");
      Serial.println(timeOfFirstButtonPress);      
    }  else {
      long dt = currentTimeMs - timeOfFirstButtonPress;
      Serial.print("time since 1st button press:");
      Serial.println(dt);
    }

  }

  timeElapsed += TIME_SLICE;
  
}


void setup()
{
  Serial.begin(115200);
  Serial.println("Entering setup() ---------------");
  delay(2000);

  Wire.begin();

  setupBuiltInButton();

  setupBuiltInNeopixel();


  setupSensors();
  Serial.println("Exiting setup() ---------------\n");

  Serial.println("Click on User/Boot button to start 5 second timer");

}

void setupBuiltInNeopixel(){
  pixel.begin(); // Initialize NeoPixel
  pixel.setBrightness(50); // Set brightness (0-255)
}


void setupBuiltInButton(){
  Serial.println("setupBuiltInButton()");
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void initLineSensors(){
    
  Serial.println("\nEnter initLineSensors() --------------------");
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){26, 27, 28}, NUM_LINE_SENSORS); // AO A1 A2
  //qtr.setEmitterPin(2);

  delay(500);


  // turn on Arduino's LED to indicate we are in calibration mode
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  Serial.println("Start calibrating all line sensors for roughly 10 seconds.");
  for (uint16_t i = 0; i < 1000; i++)
  {
    qtr.calibrate();
    delay(5);
  }
  Serial.println("End calibrating all line sensors");

  // turn off Arduino's LED to indicate we are through with calibration
  digitalWrite(LED_BUILTIN, LOW); 


  // print the calibration minimum values measured when emitters were on
  Serial.println("Min values");
  for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print('\t');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  Serial.println("Max values");
  for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print('\t');
  }
  Serial.println();
  delay(ONE_SECOND);




  Serial.println("Exit initLineSensors() ---------------------\n");  
}

// Setup both line and distance sensors using QWIIC MUX
void setupSensors() {

  initMUX();

  initDistanceSensors();

  initLineSensors();

}


void initDistanceSensors(){

  Serial.println("\nEnter initDistanceSensors() -------------");
  //Create set of pointers to the class
  distanceSensorArray = new SFEVL53L1X *[NUM_DISTANCE_SENSORS];

  //Assign pointers to instances of the class
  for (int x = 0; x < NUM_DISTANCE_SENSORS; x++){
    distanceSensorArray[x] = new SFEVL53L1X(Wire);
  }


  // Not sure if the next 3 lines are necessary for anything other than sanity check.
  byte currentPortNumber = myMux.getPort(); // initial state appears to be 255
  //Serial.print("CurrentPort: ");
  //Serial.println(currentPortNumber);

  //Initialize all the distance sensors
  bool allSensorsSuccess = true;

  for (byte port = 0; port < NUM_DISTANCE_SENSORS; port++)
  {
    myMux.setPort(port);
    int status  = distanceSensorArray[port]->begin(Wire);
    
    // Serial.print(" status");
    // Serial.print(port);
    // Serial.print("\t");
    // Serial.println(status);
    
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
      Serial.print("Distance Sensor on port ");
      Serial.print(port);
      Serial.println(" configured.");
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

  Serial.println("Exit initDistanceSensors. -------------");  
}


// 1. Handle line sensors
// 2. Handle bot sensors
void loopSensors(bool fightStarted){
  if(!fightStarted){
    return;
  }

  // line sensors have to be first, before bot sensors
  //int lineStatus = loopLineSensors();
  // if(lineStatus != LINE_NONE){
  //   return;
  // }

  //int botStatus = loopBotSensors(fightStarted);

  loopLineSensors();

}


int loopLineSensors(){
  int lineStatus = LINE_NONE;


  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
  {
    Serial.print("index:");
    Serial.print(i);
    Serial.print("\t");
    Serial.print(sensorValues[i]);
    Serial.print('\t');

    // Store the hit/miss in an array for later handling with motors.
    lineDetectedArray[i] = isLineDetected(sensorValues[i]);
  }
  Serial.print("position (hit index?):");
  Serial.println(position);


  return lineStatus;
}

bool isLineDetected(int value){
  bool isLineDetected = false;

  if(value > 850){
    isLineDetected = true;
  }

  return isLineDetected;
}

int loopBotSensors(bool fightStarted){

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
      if(fightStarted){
        if(port > 0){
          Serial.print("\t");
        }
        Serial.print("bot");
        Serial.print(port);
        Serial.print(":");
        Serial.print(botDetectedArray[port]);
      }
    } else {
      if(fightStarted){
        Serial.print("Distance");
        Serial.print(port);
        Serial.print("(cm): ");
        Serial.print(distanceCM, 2);
      }
    }

  }

  if(fightStarted){
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
  Serial.println("Mux initialized successfully.");
}

