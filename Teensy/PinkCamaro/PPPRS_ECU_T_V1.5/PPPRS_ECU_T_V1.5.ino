

//*******************************************
////////System Configuration Switcher////////
//*******************************************
#define SETCURRENTLIMIT     90//80//100// nominal 90A.  results in ~85A typical
#define OPENLOOPRAMPADJUST  1.0f //If you're in open-loop mode, this adjusts relatively how fast the ramp up occurs.  lower is safer and slower, nominal is 1.0f
//#define FIXEDSPROCKET       //UNCOMMENT THIS  TO CHOOSE YOUR OWN SPROCKET SIZE AT COMPILE-TIME

#ifdef FIXEDSPROCKET
#define LITTLESPROCKETSIZE  13//your options are 9, 10, 11, 12, or 13-tooth sprockets
#else
byte LITTLESPROCKETSIZE;
#endif


//#define DEBUGMODE         //forces a reverse button press on boot

//***************************************************/
////////
////////Camaro EMERGENCY Configuration Changes////////
////////
//***************************************************/

//IF IT JUST WONT GO (OR WONT GO FAST):

//IF FUSE IS BLOWING:
// *3:CAMARO: Turn off speed sensing in open-loop mode
boolean speedsensing =    true;
// *3:CAMARO: Slow down how fast the car ramps up in openloop mode
#define BIGSPROCKETSIZE     66.0f//your options are 54 or 66-tooth sprockets

float HIGHOPENLOOPRAMP;

//IF IT JUST WONT GO ON A GOOD FUSE:
// *1:CAMARO:  The throttle may have slipped and the range is so low that it doesnt pass THROTTLEMIN even at full pull.  or it may have slipped and it doesn't go below THROTTLEMIN on boot causing car to see a stuck throttle condition
#define THROTTLEMIN       325//default  (325, 1.05V)//Minimum ADC value that will be mapped for throttle purposes.  If the car won't go at all, turn this down.  If it still wont go, turn this back up higher than it was.  Expected Range is 150 - 500
// *2:CAMARO:  The voltage sensing may be not working, causing the car to think that the voltage is too low to run.  with this set to false, mind battery voltage.  ~10Ah per 15-min stint.
boolean voltagesensing =  true;

//IF IT WONT GO FAST:
// *1:CAMARO: Turn off speed sensing (above).  Or, every time you restart the car, hold the throttle for 2.5sec while it creeps along.  after that it will disable speed sensing automatically until reboot.
// *2:CAMARO: The throttle may have slipped and the range is so low that it doesnt pass THROTTLEMAX even at full pull.
#define THROTTLEMAX       700//default  (700, 2.26)//Maximum ADC value that will be mapped for throttle purposes.  If the car won't go fast, turn this down.  if the car goes full speed with only a little throttle applied, turn this up higher than it was.  Expected Range is 500 - 900


//INCLUDES
//these are standard teensy libraries
#include <ADC.h>
#include <EEPROM.h>
#include <FreqMeasureMulti.h>
//This is a slight modification of Arduino's built-in servo library
#include <PulseServo.h>
//my custom PulseServo library sends out a pulse whenever I write to it, not on a predefined interval like the normal servo library
//it still has a predefined interval, but that is set artificially long so that anytime I write a new value to the servo it immediately sends out a pulse of that length
//removes the variable time delay of not knowing whether you just missed the boat on a servo pulse, I now send them out completely on-demand
//Yay for latency removal!

//
//--------------------PIN DEFINITIONS
//
//digital
#define DPIN_RX1          0
#define DPIN_TX1          1
#define DPIN_ESCOUT       6
#define DPIN_BRAKE        7
#define DPIN_REVERSE      8
#define DPIN_SPEED        9
#define DPIN_BKOUT        10
#define DPIN_REVOUT       11
#define DPIN_GEARUP       5
#define DPIN_GEARDOWN     4
#define DPIN_RPMOUT       3

#define XBSERIAL1         Serial1
#define XBEESPEED         38400

//analog
#define APIN_PACKV        A0
#define APIN_THROTTLE     A1

//For sprocket pin:
//10K Bias resistor to VCC, each sprocket has a different bias resistor to GND:
//Checked once, on reset
//  Sprocket  Resistor
//  9         1K
//  10        4.7K
//  11        10K
//  12        22K
//  13        100K
#define APIN_SPROCKET     A4

//----------------------------LOOP AND SAMPLE TIMING--------------------

#define SERVOMIN              975               //LOOP AND SAMPLE TIMING  //absolute minimum value writable to the servo
#define SERVOMAX              2025              //LOOP AND SAMPLE TIMING  //absolute maximum value writable to the servo
#define SERVODEAD             1542                      //THROTTLE INPUT AND OUTPUT  //space between this and the zero band won't be sent because it will cause odd behavior with the ESC

#define MINPERIOD             2100   //LOOP AND SAMPLE TIMING  //minimum time between sending out pulses.  this enures we do not send out a new pulse before one is finished.
unsigned long prevmicros =    0;                //LOOP AND SAMPLE TIMING  //the last time we SHOULD HAVE completed a loop
unsigned long prevperiod =    0;                //LOOP AND SAMPLE TIMING  //This is the last time we ACTUALLY completed a loop
unsigned long prevmillis =    0;                //millis the last time we completed a loop
unsigned long prevsample =    0;                //the millis duration of the last sampling period.  used to calculate variable I2P


//--------------------------------THROTTLE INPUT AND OUTPUT----------------------------

unsigned int THROTTLERANGE =     THROTTLEMAX - THROTTLEMIN; //THROTTLE INPUT AND OUTPUT  //the total ADC counts difference in our min and max throttle readings
#define SERVOZERO                 1500                      //THROTTLE INPUT AND OUTPUT  //'Zero' point for servo library.
#define LIMPLEVEL                 0.25f                        //THROTTLE INPUT AND OUTPUT  //percentage of full throttle allowed under full limp home mode (low voltage)
#define BRAKEPOWER                SERVOZERO                 //THROTTLE INPUT AND OUTPUT  //amount of negative throttle to apply when we hit the brakes.  right now, it's NOTHING.  Because regen be bad for the fuse!
#define LIMPCUTOFF                230                       //THROTTLE INPUT AND OUTPUT  //if we're below this at idle, we're in maximum limp home mode
#define LIMPHYST                  260                       //THROTTLE INPUT AND OUTPUT  //if we rebound above this, take us out of limp home mode.  anything between these 2 values scales our maximum throttle linearly between normal max and <LIMPLEVEL>

boolean reversing =               0;
int traveldirection =             0;
#define GOFORWARD                 1
#define GOREVERSE                 -1

int throttlepos =               SERVOZERO;    //the throttle we calculate.  Initialize with a throttle output of zero
int testramp =                  SERVOZERO;
int lastthrottle =              SERVOZERO;    //the zero point for our servo band
boolean gearup = false;
boolean geardown = false;
boolean geardownarmed = false;
float rampremainder = 0;


//--------------------------------CURRENT LIMITING----------------
/////////////////CALIBRATION VALUES/////////////////
/////////////////CALIBRATION VALUES/////////////////
/////////////////CALIBRATION VALUES/////////////////
#if (SETCURRENTLIMIT == 90)
#define SPEEDRAMPCONSTANT     1625//90A//1620//80A//1625//100A//
#define SPEEDRAMPMULTIPLIER   0.012616f//90A//0.0121316f//80A//0.0127062//100A//
#define SPEEDRAMPCALPOINT     296//90A//304//80A//300//100A//
#elif (SETCURRENTLIMIT == 100)
#define SPEEDRAMPCONSTANT     1625//100A//1625//90A//1620//80A//
#define SPEEDRAMPMULTIPLIER   0.0127062//100A//0.012616f//90A//0.0121316f//80A//
#define SPEEDRAMPCALPOINT     300//100A//296//90A//304//80A//
#else
#define SPEEDRAMPCONSTANT     1620//80A//1625//90A//1625//100A//
#define SPEEDRAMPMULTIPLIER   0.0121316f//80A//0.012616f//90A//0.0127062//100A//
#define SPEEDRAMPCALPOINT     304//80A//296//90A//300//100A//
#endif

#define SPEEDRAMPMIN          1680
#define RPMMEETPOINT          18000.0//90A
#define SOFTBIAS              1.1f
#define AVERAGEBIAS           2.0f

/////////////////CALIBRATION VALUES/////////////////
/////////////////CALIBRATION VALUES/////////////////
/////////////////CALIBRATION VALUES/////////////////

//--------------------------------------SPEED AND ODOMETRY------------

#define SPEEDMULTIPLIER                 0.984f

float RPMMULTIPLIER;

#define SPEEDLVL1                       1000        //1000rpm

#define SPEEDTIMEOUT                    250000   //SPEED AND ODOMETRY  //number of uS to wait for a signal from the motor before declaring us stopped (0.4mph)
#define REVERSECROSSOVER                1200        //speed below which we can transfer to reverse from forward (or vice versa)
#define MAXSPEEDFILTER                  40000       //if we get a reading over 40KRPM, use our last reading instead

volatile int numtriggers =     0;        //SPEED AND ODOMETRY
unsigned long trignewtime =    0;        //SPEED AND ODOMETRY

unsigned int maxthrottle =              SERVOMAX;
boolean atmaxthrottle =                 false;
long maxthrottlestart =                 0;
#define NOSPEEDTIME                  2500      //number of mS to wait at our 'max' throttle without seeing any speed before we assume the speed sensor's broken and turn it off
unsigned int maxspeed =                 0;
#define MINLITTLESPROCKET   9
#define MAXLITTLESPROCKET   13

#define THRESHOLD9TOOTH     205
#define THRESHOLD10TOOTH    410
#define THRESHOLD11TOOTH    614
#define THRESHOLD12TOOTH    819
#define THRESHOLD13TOOTH    1010    //above this and the slot is empty, so pull from memory

#define SPROCKETADDRESS     0x10
#define CHECKSUMADDRESS     0x11

#define NUMSPEEDSAMPLES 1
float speedsamples[NUMSPEEDSAMPLES];
float rpmsamples[NUMSPEEDSAMPLES];
unsigned int speedbufferindex = 0;
unsigned long odometer = 0;

//----------------------------ENGINE SOUND------------
#define NEUTRAL 0
#define BOTTOMGEAR   1
#define TOPGEAR   3
#define SPEEDBOOTDELAY  5000
boolean ingear = false;
//                                  0th   1st     2nd     3rd     *4th
long upshiftspeeds[] =             { -1,  37000,  37000,  37000,  MAXSPEEDFILTER + 1}; //was 15000, 25000, 37000 
const long upshifthighspeeds[] =   { -1,  37000,  37000,  37000,  MAXSPEEDFILTER + 1}; //was 15000,  25000,  37000
const long upshiftlowspeeds[] =    { -1,  37000,  37000,  37000,  MAXSPEEDFILTER + 1}; //was 15000,  25000,  37000

long downshiftspeeds[] =           { -1,  0,         0,       0,  MAXSPEEDFILTER}; //was 0,      7500,   18000
const long downshifthighspeeds[] = { -1,  0,         0,       0,  MAXSPEEDFILTER}; //was 0,      7500,   18000
const long downshiftlowspeeds[] =  { -1,  0,         0,       0,  MAXSPEEDFILTER}; //was 0,      7500,   18000

const long minrpmspeeds[] =        { -1,  1800,   1800,   1800,   MAXSPEEDFILTER}; //was 1800,   3000,   4400
#define IDLERPM 1500
#define MINRPM 1550
#define MAXRPM 2025
#define HORNONSERVO 2025

#define SHIFTBLANKTIME  50
unsigned long upshifttime = 0;
unsigned long downshifttime = 0;

//----------------------------DATALOGGING AND DISPLAY------------
#define LOGGEDVARIABLES 18
long mydata[LOGGEDVARIABLES + 1];
#define   LOG_THROTIN           0  //
#define   LOG_THROTOUT          1  //
#define   LOG_VOLTAGE           2 //
#define   LOG_REVERSE           3  //
#define   LOG_BRAKE             4  //
#define   LOG_MOTOR_SPEED       5 //
#define   LOG_SPEED             6 //
#define   LOG_RPM               7  //
#define   LOG_GEAR              8
#define   SEND_BATT             9
#define   SEND_BRAKE            10
#define   SEND_REVERSE          11
#define   SEND_THROTIN          12
#define   SEND_THROTOUT         13
#define   SEND_SPEED            14
#define   LOG_LAST_MOTOR_SPEED  15 //
#define   LOG_LAST_SPEED        16 //
#define   SEND_REVBUTTON        17

unsigned int GEARDOWNTHRESHOLD = 221;

#define DATABYTESIZE  9
byte datapacket[DATABYTESIZE + 1];
#define DATAINTERVAL          31  //15fps OSD update rate, updated at least every 66.7ms.  since loops are 2ms, we will make it every 64ms to ensure we get a new update for every frame
unsigned long datamillis;
#define VOLTBUFSIZE 256
int voltagebuffer[VOLTBUFSIZE];
int voltageloop = 0;
#define NUMVSAMPLES 8;
int voltagechart[2][11] = {{4514, 4613, 4687, 4737, 4786, 4824, 4873, 4917, 4972, 5022, 5146}, {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100}};
//------------------------GENERIC VARIABLES--------------------

#define ADCBITS           10
#define ADCAVERAGING      4
#define CONVERSIONSPEED   ADC_HIGH_SPEED
#define SAMPLINGSPEED     ADC_HIGH_SPEED

float voltdivisor =       0.5161f;           //GENERIC VARIABLES  //maps our ADC-read voltage to get 100-mV resolution

float pctdivisor;                           //GENERIC VARIABLES  //maps our input uS throttle output to a percentage
float thrdivisor;                           //GENERIC VARIABLES  //maps our ADC-read throttle by this to get a uS throttle to output

unsigned long loopcounter = 0;
#define LOOPSKIP 10
//
//INSTANTIATIONS
PulseServo myservo;          //the output is basically just a servo output, specially-controlled
FreqMeasureMulti myspeed;
ADC *adc = new ADC(); // adc object;


//------------------------ANALOG SAMPLING-------------------------------------
//
//  takes 8 samples of all our analog inputs,
//  and constrains them to our relevant data ranges
//
//------------------------ANALOG SAMPLING-------------------------------------
void sample()
{
  mydata[LOG_VOLTAGE] = adc->analogRead(APIN_PACKV);
  mydata[LOG_THROTIN] = adc->analogRead(APIN_THROTTLE);

  mydata[LOG_BRAKE] = !digitalRead(DPIN_BRAKE);
  mydata[SEND_BRAKE] = mydata[LOG_BRAKE];
  mydata[LOG_REVERSE] = !digitalRead(DPIN_REVERSE);
  mydata[SEND_REVBUTTON] = mydata[LOG_REVERSE];
  if (mydata[LOG_BRAKE])
  {
    digitalWrite(DPIN_BKOUT, LOW);
  }
  else
  {
    digitalWrite(DPIN_BKOUT, HIGH);
  }
  if (mydata[LOG_REVERSE])
  {
    digitalWrite(DPIN_REVOUT, LOW);
  }
  else
  {
    digitalWrite(DPIN_REVOUT, HIGH);
  }

  //convert the current ADC readings to a numerical current


  //convert the throttle ADC readings to a servo duration
  mydata[LOG_THROTIN] = mydata[LOG_THROTIN] - THROTTLEMIN;                //invert the range (so that a disconnected pot = zero throttle)
  mydata[LOG_THROTIN] = constrain( mydata[LOG_THROTIN], 0, THROTTLERANGE); //make sure we stay in bounds
  mydata[LOG_THROTIN] *= mydata[LOG_THROTIN];
  mydata[LOG_THROTIN] /= THROTTLERANGE;
  if ((mydata[LOG_THROTIN] == THROTTLERANGE) && !gearup)
  {
    geardownarmed = true;
  }


  throttlepos = float( mydata[LOG_THROTIN]) * thrdivisor;                              //run our divider to get us in the uS range rather than ADC range
  mydata[LOG_THROTIN] = throttlepos * pctdivisor;                 //get our percent value here in the middle
  mydata[SEND_THROTIN] = mydata[LOG_THROTIN] / 10;
  if (mydata[SEND_THROTIN] < 1)
  {
    voltagebuffer[voltageloop] = mydata[LOG_VOLTAGE];
    voltageloop++;
    if (voltageloop >= VOLTBUFSIZE)
    {
      voltageloop = 0;
    }
  }

  mydata[LOG_VOLTAGE] *= voltdivisor;            //first we need to multiply by our divisor
  if (mydata[LOG_VOLTAGE] < LIMPHYST)
  {
    long templong = constrain(mydata[LOG_VOLTAGE], LIMPCUTOFF, LIMPHYST);
    int tempint = throttlepos * LIMPLEVEL;
    throttlepos = map (templong, LIMPCUTOFF, LIMPHYST, tempint, throttlepos);
  }

  throttlepos += SERVODEAD;                               //lastly, add the offset of the zero position
  if (throttlepos == SERVODEAD)
  {
    throttlepos = SERVOZERO;
  }

}

//------------------------CURRENT AND THROTTLE LIMTING-------------------------------------
//
//  uses our current values and the contents of the I2P
//  buckets to determine what our limit should be.  Then
//  we alter the input throttle to hit our current limit.
//
//------------------------CURRENT AND THROTTLE LIMTING-------------------------------------
void control()
{

  if (speedsensing)

  {
    if (voltagesensing)
    {
      float tempfloat = float(SPEEDRAMPMULTIPLIER) * float(SPEEDRAMPCALPOINT) / float(mydata[LOG_VOLTAGE]);
      if (mydata[LOG_MOTOR_SPEED] < RPMMEETPOINT)
      {
        testramp = (((tempfloat * RPMMEETPOINT) + SPEEDRAMPCONSTANT) - SPEEDRAMPMIN); //total range of values between minimum output and meet-point output
        testramp = testramp * (float(mydata[LOG_MOTOR_SPEED]) / RPMMEETPOINT); //how much of that range I am entitled to through a linear interpolation
        testramp = testramp + SPEEDRAMPMIN; //what my output would be with that range portion
        testramp = testramp * (AVERAGEBIAS - SOFTBIAS); //with a linearly interpolated output, scale it by our total minus our soft-start bias
        testramp = testramp + SOFTBIAS * ((tempfloat * mydata[LOG_MOTOR_SPEED]) + SPEEDRAMPCONSTANT); //our soft-start figure and its weighting
        testramp = testramp / AVERAGEBIAS; //weighted average of the linear figure and the soft-start figure
      }
      else
      {
        testramp = (tempfloat * mydata[LOG_MOTOR_SPEED]) + SPEEDRAMPCONSTANT;
      }
    }
    else
    {
      if (mydata[LOG_MOTOR_SPEED] < RPMMEETPOINT)
      {
        testramp = (((SPEEDRAMPMULTIPLIER * RPMMEETPOINT) + SPEEDRAMPCONSTANT) - SPEEDRAMPMIN); //total range of values between minimum output and meet-point output
        testramp = testramp * (float(mydata[LOG_MOTOR_SPEED]) / RPMMEETPOINT); //how much of that range I am entitled to through a linear interpolation
        testramp = testramp + SPEEDRAMPMIN; //what my output would be with that range portion
        testramp = testramp * (AVERAGEBIAS - SOFTBIAS); //with a linearly interpolated output, scale it by our total minus our soft-start bias
        testramp = testramp + SOFTBIAS * ((SPEEDRAMPMULTIPLIER * mydata[LOG_MOTOR_SPEED]) + SPEEDRAMPCONSTANT); //our soft-start figure and its weighting
        testramp = testramp / AVERAGEBIAS; //weighted average of the linear figure and the soft-start figure
      }
      else
      {
        testramp = (SPEEDRAMPMULTIPLIER * mydata[LOG_MOTOR_SPEED]) + SPEEDRAMPCONSTANT;
      }
    }

    if (testramp < SPEEDRAMPMIN)
    {
      testramp = SPEEDRAMPMIN;
    }

  }
  else
  {
    if (lastthrottle < SPEEDRAMPCONSTANT)
    {
      testramp = SPEEDRAMPCONSTANT;
    }
    else
    {
      testramp = lastthrottle;
    }

    rampremainder += (HIGHOPENLOOPRAMP * prevsample);

    while (rampremainder > 1)
    {
      testramp ++;
      rampremainder --;
    }

  }

  throttlepos = min (testramp, throttlepos);
  throttlepos = min (throttlepos, SERVOMAX);

  //this is set to detect a brake switch closing and choose how to apply our throttle

  if (mydata[LOG_BRAKE])
  {
    throttlepos = SERVOZERO;    //apply braking power to the RC line
    lastthrottle = SERVOZERO;       //set all our ramp functions back to zero
  }
  else
  {
    throttlepos = constrain (throttlepos, SERVOMIN, SERVOMAX);
    //make sure we're only dealing with values within the usable throttle range
    if (throttlepos > maxthrottle)
    {
      if (!atmaxthrottle)
      {
        atmaxthrottle = true;
        maxthrottlestart = millis();
      }
      else
      {
        if (speedsensing && (millis() - maxthrottlestart < NOSPEEDTIME))
        {
          throttlepos = maxthrottle;
        }
        else
        {
          speedsensing = false;
        }
      }

    }
    else
    {
      atmaxthrottle = false;
    }
  }
  lastthrottle = throttlepos;       //store this for our next time through the loop to reference
  //this catches a throttle of technically less than zero (outside SERVODEAD-SERVOMAX range) and tells it to be zero for our display
  if (throttlepos < SERVODEAD)
  {
    mydata[LOG_THROTOUT] = 0;
    throttlepos = SERVOZERO;
  }
  else
  {
    mydata[LOG_THROTOUT] = (throttlepos - SERVODEAD) * pctdivisor;  //map our current throttle reading into percent.  takes less time than the map function (mapping takes for-fricken-ever).

  }
  mydata[SEND_THROTOUT] = mydata[LOG_THROTOUT] / 10;

  mydata[SEND_REVERSE] = !mydata[LOG_REVERSE];
  if (mydata[LOG_REVERSE])                            //if the driver is holding the reverse button
  {

    if (traveldirection == GOFORWARD)                   //but if we're actually moving forward
    {
      //don't do anything differently.  the reverse button is held but we're moving forward, so our throttle should help us go forward
    }
    else                                              //if we're stopped (ready to reverse) or moving backwards already
    {
      mydata[SEND_REVERSE] = mydata[LOG_REVERSE];
      throttlepos = (3 * SERVOZERO) - (2 * throttlepos); //we invert our throttle direction
      traveldirection = GOREVERSE;                    //and let our travel direction know we're reversing
    }             //this may get reset to 0 during our calculatevalues phase but eventually we'll latch into reverse
  }
  else                                                //if the reverse button is released
  {
    if (traveldirection == GOREVERSE)                   //if we're actually moving backward
    {

      throttlepos = (3 * SERVOZERO) - (2 * throttlepos);   //don't do anything differently.  the reverse button is released but we're moving backward, so our throttle should help us go backward
    }
    else                                              //otherwise, if we're moving forward or stopped,
    {
      mydata[SEND_REVERSE] = mydata[LOG_REVERSE];
      traveldirection = GOFORWARD;                      //apply no throttle transform, and let our log know we're moving forward
    }
  }
  throttlepos - constrain (throttlepos, SERVOMIN, SERVOMAX);
  if (throttlepos == SERVOMAX)
  {
    if (!gearup)
    {
      gearup = true;
    }
  }
  if (gearup)
  {
    digitalWrite(DPIN_GEARUP, HIGH);
    digitalWrite(DPIN_GEARDOWN, LOW);
  }
  else
  {
    if (geardown)
    {
      digitalWrite(DPIN_GEARUP, LOW);
      digitalWrite(DPIN_GEARDOWN, HIGH);
    }
    else
    {
      digitalWrite(DPIN_GEARUP, LOW);
      digitalWrite(DPIN_GEARDOWN, LOW);
    }
  }

  myservo.writeMicroseconds(throttlepos);  //finally, write the throttle output
}

//VALUE CALCULATION------------------------VALUE CALCULATION-------------------------------------
// Obv.
//------------------------VALUE CALCULATION-------------------------------------

void calculatevalues()
{
  if (speedsensing)
  {
    if (myspeed.available())
    {
      if ((micros() - trignewtime) > (2 * SPEEDTIMEOUT))
      {
        float tempspeed = RPMMULTIPLIER * myspeed.countToFrequency(myspeed.read());
        mydata[LOG_LAST_MOTOR_SPEED] = 0;
        mydata[LOG_LAST_SPEED] = 0;
      }
      else
      {
        float tempspeed = myspeed.countToFrequency(myspeed.read()) * (RPMMULTIPLIER);
        mydata[LOG_LAST_SPEED] = (tempspeed * SPEEDMULTIPLIER) / float(RPMMULTIPLIER);
        if ( tempspeed < MAXSPEEDFILTER)
        {
          mydata[LOG_LAST_MOTOR_SPEED] =  tempspeed;
          if ( mydata[LOG_LAST_MOTOR_SPEED] > maxspeed)
          {
            maxspeed =  mydata[LOG_LAST_MOTOR_SPEED];
            if (geardownarmed && (mydata[LOG_LAST_SPEED] < GEARDOWNTHRESHOLD))
            {
              geardown = true;
            }
            else
            {
              geardown = false;
            }
          }
        }
      }
      trignewtime = micros();
      odometer += myspeed.readCounts();
    }
    else
    {
      if ((micros() - trignewtime) > SPEEDTIMEOUT)
      {
        mydata[LOG_LAST_MOTOR_SPEED] = 0;
        mydata[LOG_LAST_SPEED] = 0;
      }
    }
    rpmsamples[speedbufferindex] = mydata[LOG_LAST_MOTOR_SPEED];
    speedsamples[speedbufferindex] = mydata[LOG_LAST_SPEED];

    mydata[LOG_MOTOR_SPEED] = 0;
    mydata[LOG_SPEED] = 0;

    for (int i = 0; i < NUMSPEEDSAMPLES; i++)
    {
      mydata[LOG_MOTOR_SPEED] += rpmsamples[i];
      mydata[LOG_SPEED] += speedsamples[i];
    }
    mydata[LOG_MOTOR_SPEED] /= NUMSPEEDSAMPLES;
    mydata[LOG_SPEED] /= NUMSPEEDSAMPLES;
    mydata[SEND_SPEED] = 10.0 * mydata[LOG_SPEED];
    speedbufferindex++;
    if (speedbufferindex >= NUMSPEEDSAMPLES)
    {
      speedbufferindex = 0;
    }

    if (( mydata[LOG_MOTOR_SPEED] > SPEEDLVL1) || !speedsensing)
    {
      maxthrottle = SERVOMAX;
    }
    else
    {
      if (voltagesensing)
      {
        maxthrottle = SERVODEAD + (SPEEDRAMPMIN - SERVODEAD) * float(SPEEDRAMPCALPOINT) / float(mydata[LOG_VOLTAGE]) ;
      }
      else
      {
        maxthrottle = SPEEDRAMPMIN;
      }
    }
    if (traveldirection != 0)
    {
      if (mydata[LOG_MOTOR_SPEED] <= REVERSECROSSOVER)
      {
        traveldirection = 0;
      }
    }
    if (((mydata[LOG_MOTOR_SPEED] < 1)) || (((mydata[LOG_MOTOR_SPEED] < 2000) && (mydata[LOG_THROTIN] < 1))))
    {
      ingear = false;
      mydata[LOG_GEAR] = NEUTRAL;
      mydata[LOG_RPM] = map(float(mydata[LOG_THROTIN]), 0.0f, 1000.0f, MINRPM, MAXRPM);
    }
    else
    {
      upshiftspeeds[mydata[LOG_GEAR]] = map(mydata[LOG_THROTIN], 0, 1000, upshiftlowspeeds[mydata[LOG_GEAR]], upshifthighspeeds[mydata[LOG_GEAR]]);
      downshiftspeeds[mydata[LOG_GEAR]] = map(mydata[LOG_THROTIN], 0, 1000, downshiftlowspeeds[mydata[LOG_GEAR]], downshifthighspeeds[mydata[LOG_GEAR]]);
      ingear = true;
      if (mydata[LOG_GEAR] == NEUTRAL)
      {
        mydata[LOG_GEAR] = BOTTOMGEAR;
      }
      while (mydata[LOG_MOTOR_SPEED] > upshiftspeeds[mydata[LOG_GEAR]])
      {
        mydata[LOG_GEAR]++;
        upshifttime = millis();
      }
      while (mydata[LOG_MOTOR_SPEED] < downshiftspeeds[mydata[LOG_GEAR]])
      {
        mydata[LOG_GEAR]--;
        downshifttime = millis();
      }
    }
    if ((abs(mydata[LOG_THROTIN]) < 1) && (mydata[LOG_MOTOR_SPEED] < 1))
    {
      mydata[LOG_RPM] = IDLERPM;
    }
    else
    {
      if ((upshifttime + 4 * SHIFTBLANKTIME) > millis())
      {
        mydata[LOG_RPM] = IDLERPM;
      }
      else
      {
        if ((downshifttime + SHIFTBLANKTIME) > millis())
        {
          mydata[LOG_RPM] = MAXRPM;
        }
        else
        {
          if (ingear)
          {
            mydata[LOG_RPM] = map(mydata[LOG_MOTOR_SPEED], minrpmspeeds[mydata[LOG_GEAR]], upshifthighspeeds[mydata[LOG_GEAR]], MINRPM, MAXRPM);
            mydata[LOG_RPM] = constrain(mydata[LOG_RPM], MINRPM, MAXRPM);
          }
        }
      }

    }
  }
  else
  {
    if (mydata[LOG_BRAKE])
    {
      ingear = false;
      mydata[LOG_GEAR] = NEUTRAL;
      mydata[LOG_RPM] = map(float(mydata[LOG_THROTIN]), 0.0f, 1000.0f, MINRPM, MAXRPM);
    }
    else
    {
      if (abs(mydata[LOG_THROTOUT]) < 1)
      {
        mydata[LOG_RPM] = IDLERPM;
      }
      else
      {
        mydata[LOG_RPM] = MINRPM + (abs(mydata[LOG_THROTOUT]) * (MAXRPM - MINRPM)) / 1000;
      }
    }
    ingear = false;
    mydata[LOG_GEAR] = NEUTRAL;
    mydata[LOG_RPM] = map(float(mydata[LOG_THROTIN]), 0.0f, 1000.0f, MINRPM, MAXRPM);
  }
  if (millis() < SPEEDBOOTDELAY)
  {
    mydata[LOG_RPM] = max(mydata[LOG_RPM], MINRPM);
  }
}

void sendData()
{
  if (mydata[SEND_THROTOUT] < 1)
  {
    long templong = 0;
    int tempint = 0;
    for (int i = 0; i < VOLTBUFSIZE; i++)
    {
      templong += voltagebuffer[i];
    }
    templong *= NUMVSAMPLES;
    templong /= VOLTBUFSIZE;
    templong = constrain(templong, voltagechart[0][0], voltagechart[0][10]);
    for (int i = 0; i < 10; i++)
    {
      if (templong <= voltagechart[0][i + 1])
      {
        tempint = i;
        break;
      }

    }
    templong = map(templong, voltagechart[0][tempint], voltagechart[0][tempint + 1], voltagechart[1][tempint], voltagechart[1][tempint + 1]);
    mydata[SEND_BATT] = templong;
  }

  datapacket[0] = (mydata[SEND_BRAKE] << 7 | (mydata[SEND_THROTIN] & 0x7F));
  datapacket[1] = (mydata[SEND_REVERSE] << 7 | (mydata[SEND_THROTOUT] & 0x7F));
  datapacket[2] = (mydata[SEND_REVBUTTON] << 7 | (mydata[SEND_BATT] & 0x7F));
  datapacket[3] = (mydata[SEND_SPEED] >> 8) & 0xFF;
  datapacket[4] = mydata[SEND_SPEED] & 0xFF;
  long templong = (odometer * float(11.0 * 3.14)) / 20;
  datapacket[5] = (templong >> 16) & 0xFF;
  datapacket[6] = (templong >> 8) & 0xFF;
  datapacket[7] = templong & 0xFF;
  datapacket[8] = 0xFF;

  for (int i = 0; i < DATABYTESIZE; i++)
  {
    XBSERIAL1.write(datapacket[i]);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////MAIN FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////MAIN FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////MAIN FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////MAIN FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////MAIN FUNCTIONS

void setup()                                             // run once, when the sketch starts
{
  adc->setAveraging(ADCAVERAGING); // set number of averages
  adc->setResolution(ADCBITS); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed


  byte outpins[] = {DPIN_RPMOUT, DPIN_ESCOUT, DPIN_GEARUP, DPIN_GEARDOWN, DPIN_BKOUT, DPIN_REVOUT};      //all output pins
  for ( int i = 0; i < sizeof(outpins); i++) {
    pinMode(outpins[i], OUTPUT);                             //turn to output
    digitalWrite(outpins[i], LOW);
  }

  //set our inputs
  byte inpins[] = {DPIN_SPEED};         //all input pins
  for ( int i = 0; i < sizeof(inpins); i++) {
    pinMode(inpins[i], INPUT);                            //set tham all to input
  }

  byte pulluppins[] = {DPIN_BRAKE, DPIN_REVERSE};        //all input pins with pullups
  for ( int i = 0; i < sizeof(pulluppins); i++) {
    pinMode(pulluppins[i], INPUT_PULLUP);                            //set tham all to input with pullup
  }

  //Most important thing is making sure the controller is set to zero throttle when it boots.  seriously, there are protections in the ESC but redundancy is always good for safety
  myservo.attach(DPIN_ESCOUT, SERVOMIN, SERVOMAX);
  myservo.writeMicroseconds(SERVOZERO);

#ifndef FIXEDSPROCKET
  unsigned int tempunint = adc->analogRead(APIN_SPROCKET);
  LITTLESPROCKETSIZE = MAXLITTLESPROCKET;
  if (tempunint < THRESHOLD9TOOTH)
  {
    LITTLESPROCKETSIZE = 9;
  }
  else
  {
    if (tempunint < THRESHOLD10TOOTH)
    {
      LITTLESPROCKETSIZE = 10;
    }
    else
    {
      if (tempunint < THRESHOLD11TOOTH)
      {
        LITTLESPROCKETSIZE = 11;
      }
      else
      {
        if (tempunint < THRESHOLD12TOOTH)
        {
          LITTLESPROCKETSIZE = 12;
        }
        else
        {
          LITTLESPROCKETSIZE = 13;
          if (tempunint > THRESHOLD13TOOTH)
          {
            byte tempbyte1 = EEPROM.read(SPROCKETADDRESS);
            byte tempbyte2 = EEPROM.read(CHECKSUMADDRESS);
            if ((tempbyte1 + tempbyte2) == 0xff)
            {
              LITTLESPROCKETSIZE = tempbyte1;
            }
          }
        }
      }
    }
  }
  LITTLESPROCKETSIZE = constrain(LITTLESPROCKETSIZE, MINLITTLESPROCKET, MAXLITTLESPROCKET);

  EEPROM.write(SPROCKETADDRESS, LITTLESPROCKETSIZE);
  EEPROM.write(CHECKSUMADDRESS, ~LITTLESPROCKETSIZE);

#endif

  HIGHOPENLOOPRAMP = OPENLOOPRAMPADJUST * 0.31625 * float(LITTLESPROCKETSIZE) / BIGSPROCKETSIZE;
  RPMMULTIPLIER = 24.5f * BIGSPROCKETSIZE / float(LITTLESPROCKETSIZE); //((60s/m * 8.1667:1gear / 20poles) * GEARING )

  if (LITTLESPROCKETSIZE == 9)       //Full output throttle for this ratio is 17.4mph (top speed is 20.3mph)
  {
    GEARDOWNTHRESHOLD = 144;       //14.4mph is 3mph below our full output speed
  }
  else
  {
    if (LITTLESPROCKETSIZE == 10)    //Full output throttle for this ratio is 19.3mph (top speed is 22.5mph)
    {
      GEARDOWNTHRESHOLD = 164;      //16.4mph is 1mph below the next lowest gear's full throttle speed
    }
    else
    {
      if (LITTLESPROCKETSIZE == 11)    //Full output throttle for this ratio is 21.2mph (top speed is 24.7mph)
      {
        GEARDOWNTHRESHOLD = 183 ;     //18.3mph is 1mph below the next lowest gear's full throttle speed
      }
      else
      {
        if (LITTLESPROCKETSIZE == 12)    //Full output throttle for this ratio is 23.1mph (top speed is 26.9mph)
        {
          GEARDOWNTHRESHOLD = 202;      //20.2mph is 1mph below the next lowest gear's full throttle speed
        }
        else
        {
          //(LITTLESPROCKETSIZE == 13)  //Full output throttle for this ratio is 25.1mph (top speed is 29.2mph)
          GEARDOWNTHRESHOLD = 221;      //22.1mph is 1mph below the next lowest gear's full throttle speed
        }
      }
    }
  }

  XBSERIAL1.begin(XBEESPEED);
#ifdef DEBUGMODE
  Serial.begin(38400);
  while (digitalRead(DPIN_BRAKE))
  {
    Serial.print(millis(), DEC);
    Serial.print(" : ");
    Serial.print("Sprocket Teeth: ");
    Serial.println(LITTLESPROCKETSIZE);
    Serial.println("Waiting for BRAKE Press");
    delay(1000);
  }
  while (!digitalRead(DPIN_BRAKE))
  {
    Serial.print(millis(), DEC);
    Serial.print(" : ");
    Serial.println("Waiting for BRAKE Release");
    delay(1000);
  }
  Serial.println("Begin");
#endif

  pctdivisor = 1000.0f / (SERVOMAX - SERVODEAD);
  thrdivisor = float(SERVOMAX - SERVODEAD) / float(THROTTLERANGE);

  //if the controller boots up and the throttle is held down, wait here until it returns to zero
  if (analogRead(APIN_THROTTLE) > THROTTLEMIN)
  {
#ifdef DEBUGMODE
    Serial.print(millis(), DEC);
    Serial.print(" : ");
    Serial.println("Waiting for safe throttle");
#endif
    while (analogRead(APIN_THROTTLE) > THROTTLEMIN)
    {

    }
  }
  numtriggers = 0;

  myspeed.begin(DPIN_SPEED, FREQMEASUREMULTI_RAISING);

  delay(100);

#ifdef DEBUGMODE
  Serial.print(millis(), DEC);
  Serial.print(" : ");
  Serial.println("GO!");
#endif
  //Our loop timing reference starts now.  GO!
  prevmillis = millis();
}

void loop() {                                            //main loop
  sample();
  calculatevalues();
  control();

#ifdef DEBUGMODE
  loopcounter++;
  if (!(loopcounter % LOOPSKIP))
  {
    //Serial.print (mydata[LOG_MOTOR_SPEED]);
    Serial.print (", ");
    Serial.println (mydata[LOG_SPEED]);
  }
#endif
  //pause here to enforce our loop timing
  if ((millis() - datamillis) > DATAINTERVAL)
  {
    datamillis = millis();
    sendData();
  }
  while ((micros() - prevperiod) < MINPERIOD)
  {
#ifndef DEBUGMODE
    unsigned long mytime = micros();
    if (mytime >= (prevperiod + mydata[LOG_RPM]))
    {
      digitalWrite(DPIN_RPMOUT, LOW);
    }
#endif
  }
  prevsample = millis() - prevmillis;
  prevmillis = millis();
  digitalWrite(DPIN_RPMOUT, HIGH);
  prevperiod = micros();
}
