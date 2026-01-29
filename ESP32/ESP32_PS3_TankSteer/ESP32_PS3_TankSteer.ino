// RC Controlled Tank Steer robot
// Handle both DRV8833 or DRV8871

#include <Ps3Controller.h>      // Needs the "Fork of PS3 Controller Host" not the "PS3 Controller Host" library
#include <FS_MX1508.h>          // For DRV8871 motor driver. DRV8833 does not need a special class.

#define PS3_BLACK_BLACK_1   "00:19:c1:c2:d8:01"
#define PS3_BLACK_BLACK_2   "00:19:c1:c2:d8:02" 
#define PS3_BLACK_BLACK_3   "00:19:c1:c2:d8:03" 
#define PS3_BLUE_BLACK_1    "00:19:c1:c2:ee:01"
int ps3Player = 0;
int ps3Battery = 0;

// -100% to 100%.
// Using percent because different motor controllers have different ranges.
long joystickLeftPctX = 0;
long joystickLeftPctY = 0;
long joystickRightPctX = 0;
long joystickRightPctY = 0;

#define MAX_PWM_VALUE 255

// Only use one of the next 2 lines to choose which motor controller we are using.
#define USE_DRV8833 1
//#define USE_DRV8871 1

// DRV8833 Motor Controller. One breakout board can control 2 motors.
#define MOT_A1_PIN 2   // og 10
#define MOT_A2_PIN 4   // og 9
#define MOT_B1_PIN 18    // og 6
#define MOT_B2_PIN 19    // og 5

// DRV8871 Motor controllers. One for each side.
// Each motor needs 2 pins.
//@TODO set 4 unique pins to control the 2 motors
#define PIN_MOTOR_L_A 18 // Left Motor
#define PIN_MOTOR_L_B 19 // Left Motor
#define PIN_MOTOR_R_A 18 // Right Motor
#define PIN_MOTOR_R_B 19 // Right Motor
#define MAX_MOTOR_PCT 100 // Input is -100 to +100
MX1508 motorL(PIN_MOTOR_L_A, PIN_MOTOR_L_B); // default SLOW_DECAY (resolution 8 bits, frequency 1000Hz)
MX1508 motorR(PIN_MOTOR_R_A, PIN_MOTOR_R_B); // default SLOW_DECAY (resolution 8 bits, frequency 1000Hz)




#define SLP_PIN 13

#define TIME_SLICE_MS 100 // A tenth of a second, cycle time.

void notifyPS3Controller()
{

    printPS3ButtonRawValues();
    //printJoystickRawValues();
    handleJoystickChanges();



    //---------------------- Battery events ---------------------
    printBatteryStatus();

}

void printPS3ButtonRawValues(){

    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.event.button_down.cross ) {
        Serial.println("Started pressing the cross button");
    }
    if( Ps3.event.button_up.cross ){
        Serial.println("Released the cross button");
    }

    if( Ps3.event.button_down.square ){
        Serial.println("Started pressing the square button");
    }
    if( Ps3.event.button_up.square ) {
        Serial.println("Released the square button");
    }
    if( Ps3.event.button_down.triangle ) {
        Serial.println("Started pressing the triangle button");
    }
    if( Ps3.event.button_up.triangle ) {
        Serial.println("Released the triangle button");
    }

    if( Ps3.event.button_down.circle ) {
        Serial.println("Started pressing the circle button");
    }
    if( Ps3.event.button_up.circle ) {
        Serial.println("Released the circle button");
    }

    //--------------- Digital D-pad button events --------------
    if( Ps3.event.button_down.up ) {
        Serial.println("Started pressing the up button");
    }
    if( Ps3.event.button_up.up ) {
        Serial.println("Released the up button");
    }

    if( Ps3.event.button_down.right ) {
        Serial.println("Started pressing the right button");
    }
    if( Ps3.event.button_up.right ) {
        Serial.println("Released the right button");
    }

    if( Ps3.event.button_down.down ) {
        Serial.println("Started pressing the down button");
    }
    if( Ps3.event.button_up.down ) {
        Serial.println("Released the down button");
    }

    if( Ps3.event.button_down.left ) {
        Serial.println("Started pressing the left button");
    }
    if( Ps3.event.button_up.left ) {
        Serial.println("Released the left button");
    }

    //------------- Digital shoulder button events -------------
    if( Ps3.event.button_down.l1 ) {
        Serial.println("Started pressing the left shoulder button");
    }
    if( Ps3.event.button_up.l1 ) {
        Serial.println("Released the left shoulder button");
    }

    if( Ps3.event.button_down.r1 ) {
        Serial.println("Started pressing the right shoulder button");
    }
    if( Ps3.event.button_up.r1 ) {
        Serial.println("Released the right shoulder button");
    }

    //-------------- Digital trigger button events -------------
    if( Ps3.event.button_down.l2 ) {
        Serial.println("Started pressing the left trigger button");
    }
    if( Ps3.event.button_up.l2 ) {
        Serial.println("Released the left trigger button");
    }

    if( Ps3.event.button_down.r2 ) {
        Serial.println("Started pressing the right trigger button");
    }
    if( Ps3.event.button_up.r2 ) {
        Serial.println("Released the right trigger button");
    }

    //--------------- Digital stick button events --------------
    if( Ps3.event.button_down.l3 ) {
        Serial.println("Started pressing the left stick button");
    }
    if( Ps3.event.button_up.l3 ) {
        Serial.println("Released the left stick button");
    }

    if( Ps3.event.button_down.r3 ) {
        Serial.println("Started pressing the right stick button");
    }
    if( Ps3.event.button_up.r3 ) {
        Serial.println("Released the right stick button");
    }

    //---------- Digital select/start/ps button events ---------
    if( Ps3.event.button_down.select ) {
        Serial.println("Started pressing the select button");
    }
    if( Ps3.event.button_up.select ) {
        Serial.println("Released the select button");
    }

    if( Ps3.event.button_down.start ) {
        Serial.println("Started pressing the start button");
    }
    if( Ps3.event.button_up.start ) {
        Serial.println("Released the start button");
    }
    if( Ps3.event.button_down.ps ) {
        Serial.println("Started pressing the Playstation button");
    }
    if( Ps3.event.button_up.ps ) {
        Serial.println("Released the Playstation button");
    }
}




void handleJoystickChanges(){
    // Left Stick
   if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 5 ){
       Serial.print("Left stick:");

        long rawX = Ps3.data.analog.stick.lx;
        long rawY = Ps3.data.analog.stick.ly;

        // Want the output from -100% to + 100%.
        // We can use this to convert to whatever value the motor controller is expecting.
        joystickLeftPctX = map(rawX, -127, 127, -100, 100);
        joystickLeftPctY = map(rawY, 127, -127, -100, 100);

        Serial.print(" joystickLeftPctY="); Serial.print(joystickLeftPctY, DEC); Serial.print(" rawY="); Serial.print(rawY, DEC);
        Serial.println();

    }

    // Right Stick
   if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 5 ){
        Serial.print("Right stick:");

        long rawX = Ps3.data.analog.stick.rx;
        long rawY = Ps3.data.analog.stick.ry;

        // Want the output from -100% to + 100%.
        // We can use this to convert to whatever value the motor controller is expecting.
        joystickRightPctX = map(rawX, -127, 127, -100, 100);
        joystickRightPctY = map(rawY, 127, -127, -100, 100);

        Serial.print(" joystickRightPctY="); Serial.print(joystickRightPctY, DEC); Serial.print(" rawY="); Serial.print(rawY, DEC);
        Serial.println();

   }
}


void printPS3ButtonCombos(){
    //------ Digital cross/square/triangle/circle buttons ------
    if( Ps3.data.button.cross && Ps3.data.button.down )
        Serial.println("Pressing both the down and cross buttons");
    if( Ps3.data.button.square && Ps3.data.button.left )
        Serial.println("Pressing both the square and left buttons");
    if( Ps3.data.button.triangle && Ps3.data.button.up )
        Serial.println("Pressing both the triangle and up buttons");
    if( Ps3.data.button.circle && Ps3.data.button.right )
        Serial.println("Pressing both the circle and right buttons");

    if( Ps3.data.button.l1 && Ps3.data.button.r1 )
        Serial.println("Pressing both the left and right bumper buttons");
    if( Ps3.data.button.l2 && Ps3.data.button.r2 )
        Serial.println("Pressing both the left and right trigger buttons");
    if( Ps3.data.button.l3 && Ps3.data.button.r3 )
        Serial.println("Pressing both the left and right stick buttons");
    if( Ps3.data.button.select && Ps3.data.button.start )
        Serial.println("Pressing both the select and start buttons");

}

void printJoystickRawValues(){
    // Left Stick
   if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 5 ){
       Serial.print("Moved the left stick:");

        long rawX = Ps3.data.analog.stick.lx;
        long rawY = Ps3.data.analog.stick.ly;

        long jsX = map(rawX, -127, 127, -100, 100);
        long jsY = map(rawY, 127, -127, -100, 100);

        Serial.print(" percentX="); Serial.print(jsX, DEC); Serial.print(" rawX="); Serial.print(rawX, DEC);
        Serial.print(" percentY="); Serial.print(jsY, DEC); Serial.print(" rawY="); Serial.print(rawY, DEC);
        Serial.println();

    }

    // Right Stick
   if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 5 ){
        Serial.print("Moved the right stick:");

        long rawX = Ps3.data.analog.stick.rx;
        long rawY = Ps3.data.analog.stick.ry;

        long jsX = map(rawX, -127, 127, -100, 100);
        long jsY = map(rawY, 127, -127, -100, 100);

        Serial.print(" percentX="); Serial.print(jsX, DEC); Serial.print(" rawX="); Serial.print(rawX, DEC);
        Serial.print(" percentY="); Serial.print(jsY, DEC); Serial.print(" rawY="); Serial.print(rawY, DEC);
        Serial.println();

   }
}

void printBatteryStatus(){
    if( ps3Battery != Ps3.data.status.battery ){
        ps3Battery = Ps3.data.status.battery;
        Serial.print("PS3 Controller battery is ");
        if( ps3Battery == ps3_status_battery_charging )      Serial.println("CHARGING");
        else if( ps3Battery == ps3_status_battery_full )     Serial.println("FULL");
        else if( ps3Battery == ps3_status_battery_high )     Serial.println("HIGH");
        else if( ps3Battery == ps3_status_battery_low)       Serial.println("LOW");
        else if( ps3Battery == ps3_status_battery_dying )    Serial.println("DYING");
        else if( ps3Battery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
        else Serial.println("UNDEFINED");
    }
}

void onConnectPS3Connect(){
    Serial.println("PS3 Controller Connected.");
}

void setup()
{
    Serial.println("Setup() started");

    Serial.begin(115200);

    setupMotors();

    setupPS3Controller(); 

    Serial.println("Setup() ended");
}

void setupPS3Controller() {

    Ps3.attach(notifyPS3Controller);
    Ps3.attachOnConnect(onConnectPS3Connect);

    Ps3.begin(PS3_BLACK_BLACK_1);

    Ps3.setPlayer(ps3Player);

    //-------------------- Player LEDs -------------------
    Serial.print("Setting LEDs to player "); Serial.println(ps3Player, DEC);

}



void setupMotors(void)
{
  // Set all the motor control inputs to OUTPUT
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);

  pinMode(SLP_PIN, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

  digitalWrite(SLP_PIN, HIGH);

}

void loop()
{
    if(!Ps3.isConnected()){
        return;
    }


    loopMotors();



    delay(TIME_SLICE_MS);
}

void loopMotors(){


    // For this controller system, we only care about the up/down Axis.
#ifdef USE_DRV8833
    // Expecting calculated range -255 to 255
    int pwmLeftX, pwmLeftY;
    int pwmRightX, pwmRightY;

    pwmLeftX = (joystickLeftPctX * MAX_PWM_VALUE)/100;
    pwmLeftY = (joystickLeftPctY * MAX_PWM_VALUE)/100;

    pwmRightX = (joystickRightPctX * MAX_PWM_VALUE)/100;
    pwmRightY = (joystickRightPctY * MAX_PWM_VALUE)/100;

    setMotorPWMs(pwmLeftY, pwmRightY);
#endif

#ifdef USE_DRV8871
    // Can use the calculated up/down Percent Y values for each joystick.
    driveMotors(joystickLeftPctY, joystickRightPctY);
#endif


}

// DRV8871 motor controller
void driveMotors(int pctL, int pctR){

  if(pctL > MAX_MOTOR_PCT){
    pctL = MAX_MOTOR_PCT;
  }
  else if(pctL < -MAX_MOTOR_PCT){
    pctL = -MAX_MOTOR_PCT;
  }
  if(pctR > MAX_MOTOR_PCT){
    pctR = MAX_MOTOR_PCT;
  }
  else if(pctR < -MAX_MOTOR_PCT){
    pctR = -MAX_MOTOR_PCT;
  }

  motorL.motorGoP(pctL);
  motorR.motorGoP(pctR);

}


/// Set the current on a motor channel using PWM and directional logic.
/// DRV8831 motor controller
/// \param pwm    PWM duty cycle ranging from -255 full reverse to 255 full forward
/// \param IN1_PIN  pin number xIN1 for the given channel
/// \param IN2_PIN  pin number xIN2 for the given channel
void setMotorPWM(int pwm, int IN1_PIN, int IN2_PIN)
{
    if(pwm < -MAX_PWM_VALUE){
        pwm = -MAX_PWM_VALUE;
    } else if(pwm > MAX_PWM_VALUE){
        pwm = MAX_PWM_VALUE;
    }


    if (pwm < 0) {  // reverse speeds
        analogWrite(IN1_PIN, -pwm);
        digitalWrite(IN2_PIN, LOW);
    } else { // stop or forward
        digitalWrite(IN1_PIN, LOW);
        analogWrite(IN2_PIN, pwm);
    }
}

/// Set the current on both motors.
/// DRV8831 motor controller
/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255
void setMotorPWMs(int pwm_A, int pwm_B)
{
    // Make sure input PWM values are within correct range.
    if(pwm_A < -MAX_PWM_VALUE){
        pwm_A = -MAX_PWM_VALUE;
    } else if(pwm_A > MAX_PWM_VALUE){
        pwm_A = MAX_PWM_VALUE;
    }
    if(pwm_B < -MAX_PWM_VALUE){
        pwm_B = -MAX_PWM_VALUE;
    } else if(pwm_B > MAX_PWM_VALUE){
        pwm_B = MAX_PWM_VALUE;
    }

  setMotorPWM(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  setMotorPWM(pwm_B, MOT_B1_PIN, MOT_B2_PIN);

  // Print a status message to the console.
  Serial.print("Set motor A PWM = ");
  Serial.print(pwm_A);
  Serial.print(" motor B PWM = ");
  Serial.println(pwm_B);
}

