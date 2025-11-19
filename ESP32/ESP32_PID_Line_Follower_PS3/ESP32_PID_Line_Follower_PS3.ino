/*
 * Starting code opied from this project: https://projecthub.arduino.cc/anova9347/line-follower-robot-with-pid-controller-01813f
 * by Carlos Garcia 11/16/2025
 * Modified to use different sensor array and motor driver

 * File name: PID_LF_example
 * 
 * Original Hardware requirements:
  an Arduino Pro Mini
  a QTR-8RC Reflectance Sensor Array
 a DRV8835 Dual Motor Driver Carrier 
 *                        

 * Description: The basic PID control system implemented with the line follower with the specified hardware. 
 * The robot can follow a black line on a white surface 
 *              (or vice versa). 
 * Related Document: See the written documentation or the LF video from Bot Reboot.
 *                   
 * Author: Bot Reboot
 */

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"         // Serial Controlled Motor Driver
#include "SCMD_config.h"  // Contains #defines for common SCMD register names and values
#include "Wire.h"         // For I2C/QWIIC/STEMMA Communication.



#define LED_BUILTIN 2 // works with ESP32 DEV board, Acebott ESP32-Max
#define CENTER_LINE_VAL 3500


// Sensor Array defined below
#define SENSOR_OUTER_LEFT   1
#define SENSOR_INNER_LEFT   0
#define SENSOR_CENTER       2
#define SENSOR_INNER_RIGHT  3
#define SENSOR_OUTER_RIGHT   4


// Convert sensor values to a relative position of line.
// Arbitrary scale from left to right of 0-7000, center 3500
#define SENSOR_POS_MIN 0
#define SENSOR_POS_CENTER 3500
#define SENSOR_POS_MAX 7000
#define SENSOR_POS_16P    1166
#define SENSOR_POS_32P    2331
#define SENSOR_POS_50P    3500
#define SENSOR_POS_66P    4277
#define SENSOR_POS_83P    5833
#define SENSOR_POS_100P   7000

int sensorPosition = SENSOR_POS_CENTER; // A single value used represent line relative to sensor array.


// For QWIIC Motor Driver
#define DIR_FW  0
#define DIR_RV  1
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
SCMD myMotorDriver; //This creates the main object of one motor driver and connected peripherals.



/*************************************************************************
*
  Sensor Array object initialisation 
*************************************************************************/
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int irPins[SensorCount] = {A4, A3, A2, A1,A0}; // Added by Carlos



/*************************************************************************
*
  PID control system variables 
*************************************************************************/
float Kp = 0;  //related to the proportional control term; change the value by trial - and-error(ex : 0.07).
float Ki = 0;  //related to the integral control term; change the value by trial-and-error (ex: 0.0008).
float Kd = 0;  //related to the derivative control term; change the value by trial - and-error(ex : 0.6).
int P;
int I;
int D;

/*************************************************************************
*
  Global variables
*************************************************************************/
int lastError = 0;
boolean onoff = false;

/*************************************************************************
* Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;


/*************************************************************************
* Buttons pins declaration
*************************************************************************/
int pinButtonCalibrate = 17;  //or pin A3
int pinButtonStart = 2;

/*************************************************************************
* Function Name: setup
**************************************************************************
* Summary:
* This is the setup function for the Arduino board. It first sets up the 
* pins for the sensor array and the motor driver. Then the user needs to
* slide the sensors across the line for 10 sec onds as they need to be 
* calibrated. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void setup() {

  Serial.begin(115200);

  setupLineSensors();

  setupMotors();

  setupReadouts();

  delay(500);



  /*  Not sure this should be in setup()
  boolean startInitiated = false;
  while (startInitiated == false) {  // the main function won't start until the robot is calibrated
    if (isCalibrateButtonPressed()) {
      calibrateSensorArray();  //calibrate the robot for 10 seconds
      startInitiated = true;
    }
  }
  */

  motorsAllStop();
}


void calibrateSensorArray(){
  // TODO Calibrate OSOYOO array here
}

boolean isCalibrateButtonPressed(){
  return (digitalRead(pinButtonCalibrate) == HIGH); 
}

void motorsAllStop(){
  //TODO put hardware specific code here
  myMotorDriver.setDrive( LEFT_MOTOR, DIR_FW, 0);
  myMotorDriver.setDrive( RIGHT_MOTOR, DIR_FW, 0);
}

void setupReadouts(){
  pinMode(LED_BUILTIN, OUTPUT);
}

void setupMotors(){

  setupQwiicMotorDriver();


}



void setupLineSensors(){
  // OSOYOO setup here
  setupOsoyooSensorArray();
}


void setupOsoyooSensorArray(){
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  Serial.println("setupOsoyooSensorArray() complete.");
}


/*************************************************************************
*
  Function Name: loop
**************************************************************************
* Summary:
* This is the main function of this application. When the start button is
* pressed, the robot will toggle between following the track and stopping.
* When following the track, the function calls the PID control method. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void loop() {
  if (isStartButtonPressed()) {
    onoff = !onoff;
    if (onoff = true) {
      delay(1000);  //a delay when the robot starts
    } else {
      delay(50);
    }
  }

  if (onoff == true) {
    PIDControl();
  } else {
    motorsAllStop();   //stop the motors
  }
}

boolean isStartButtonPressed(){
  return digitalRead(pinButtonStart) == HIGH;
}


// Hardware used is hidden
uint16_t getSensorArrayPosition(){
  // TODO get OSOYOO value here
  return getOsoyooSensorPosition();
}

/*************************************************************************
*
  Function Name: PID_control
**************************************************************************
* Summary: 
* This is the function of the PID control system. The distinguishing
* feature of the PID controller is the ability to use the three control 
* terms of proportional, integral and derivative influence on the controller 
* output to apply accurate and optimal control. This correction is applied to
* the speed of the motors, which should be in range of the interval [0, max_speed],
* max_speed <= 255. 
* 
* Parameters:
* none
* 
* Returns:
*  none
*************************************************************************/
void PIDControl() {

  uint16_t position = getSensorArrayPosition(); // Returns value between 0-7000
  int error = CENTER_LINE_VAL - position;       // 3500 is the ideal position (the centre)

  P = error;
  I = I + error;
  D = error - lastError;

  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;  //calculate the correction needed to be applied to the speed


  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  // Make sure values are within min and max bounds
  motorspeeda = getMotorSpeedWithinBounds(motorspeeda);
  motorspeedb = getMotorSpeedWithinBounds(motorspeedb);

  motorController(motorspeeda, motorspeedb);
}

int getMotorSpeedWithinBounds(int speed){
  int newSpeed = speed;
  if (speed > maxspeedb) {
      newSpeed = maxspeedb;
  }
  else if (speed < 0) {
    newSpeed = 0;
  }

  return newSpeed;
}

void motorController(uint16_t motorSpeedA, uint16_t motorSpeedB){

  qwiicMotorController(motorSpeedA, motorSpeedB);
}


void qwiicMotorController(uint16_t motorSpeedA, uint16_t motorSpeedB){
  myMotorDriver.setDrive( LEFT_MOTOR, DIR_FW,   motorSpeedA);
  myMotorDriver.setDrive( RIGHT_MOTOR, DIR_FW,  motorSpeedB);
}



int getOsoyooSensorPosition(){

  // Get the raw binary values from the 5 sensor array
  int numSensorHits=0;
  for(int i=0 ; i<SensorCount ;  i++){
    sensorValues[i] = digitalRead(irPins[i]);
    if(sensorValues[i]){
      numSensorHits++;  
    }        
  }
  
    if(numSensorHits == 0){
      // No sensors, real bad
      Serial.println("zero sensor hits. Bad if we are on a track racing.");
    } else if(numSensorHits == 1){
      if(sensorValues[SENSOR_OUTER_LEFT]){
        sensorPosition = SENSOR_POS_16P;
        Serial.print("1 Outer left:"); Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_INNER_LEFT]){
        sensorPosition = SENSOR_POS_32P;
        Serial.print("1 Inner left:");Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_CENTER]){
        sensorPosition = SENSOR_POS_CENTER;
        Serial.print("1 Center:");Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_INNER_RIGHT]){
        sensorPosition = SENSOR_POS_66P;
        Serial.print("1 Inner right:");Serial.println(sensorPosition, DEC);
        
      } else if(sensorValues[SENSOR_OUTER_RIGHT]){
        sensorPosition = SENSOR_POS_83P;
        Serial.print("1 Outer right:");Serial.println(sensorPosition, DEC);        
      }
  } else if(numSensorHits == 2){
    if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT]){
      sensorPosition = SENSOR_POS_32P;
      Serial.print("2 Hard left:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_POS_16P;
      Serial.print("2 Hard left:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_POS_MIN;
      Serial.print("2 Hard left:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      Serial.print("2 Hard right:");Serial.println(sensorPosition, DEC);
      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      Serial.print("2 Hard right:");Serial.println(sensorPosition, DEC);
    } else if(sensorValues[SENSOR_INNER_RIGHT] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      Serial.print("2 Hard right:");Serial.println(sensorPosition, DEC);      
    }
  } else if(numSensorHits >= 3){
    // intersection or end zone
    sensorPosition = SENSOR_POS_CENTER;
    if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_INNER_RIGHT]){
      sensorPosition = SENSOR_POS_CENTER;
      Serial.print("3+ Perpendicular line or end solid shape:");Serial.println(sensorPosition, DEC);      
    } else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_LEFT] && sensorValues[SENSOR_OUTER_LEFT]){
      sensorPosition = SENSOR_POS_MIN;
      Serial.print("3 Hard left:"); Serial.println(sensorPosition, DEC);
    }  else if(sensorValues[SENSOR_CENTER] && sensorValues[SENSOR_INNER_RIGHT] && sensorValues[SENSOR_OUTER_RIGHT]){
      sensorPosition = SENSOR_POS_MAX;
      Serial.print("3 Hard right:");Serial.println(sensorPosition, DEC);
    } else {
      sensorPosition = SENSOR_POS_CENTER;
      Serial.println("3+ Perpendicular line or end solid shape:"); Serial.println(sensorPosition, DEC);
    }

    return sensorPosition;    
  }

}

void setupQwiicMotorDriver(){
  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face is I2C_MODE 
  myMotorDriver.settings.commInterface = I2C_MODE;

  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern is "1000" (default) on board for address 0x5D

  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;

  //*****initialize the driver get wait for idle*****//
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.print("begin() return word ");
    Serial.print(myMotorDriver.begin(), HEX);
    Serial.print(" : ");
    Serial.println( "ID mismatch, trying again" );    
    delay(500);
  }
  Serial.println( "ID matches 0xA9" );

  //  Check to make sure the driver is done looking for peripherals before beginning
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");
  Serial.println();

  //*****Set application settings and enable driver*****//

  //Uncomment code for motor 0 inversion
  //while( myMotorDriver.busy() );
  //myMotorDriver.inversionMode(0, 1); //invert motor 0

  //Uncomment code for motor 1 inversion
  while ( myMotorDriver.busy() ); //Waits until the SCMD is available.
  myMotorDriver.inversionMode(1, 1); //invert motor 1

  while ( myMotorDriver.busy() )
    ; // Do nothing until driver is available

  myMotorDriver.enable(); //Enables the output driver hardware

}



