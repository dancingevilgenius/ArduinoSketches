

#include <Ps3Controller.h> // Needs the "Fork of PS3 Controller Host" not the "PS3 Controller Host" library

#define PS3_BLACK_BLACK_1   "00:19:c1:c2:d8:01"
#define PS3_BLACK_BLACK_2   "00:19:c1:c2:d8:02" 
#define PS3_BLACK_BLACK_3   "00:19:c1:c2:d8:03" 
#define PS3_BLUE_BLACK_1    "00:19:c1:c2:ee:01"

int player = 0;
int battery = 0;


// Define the control inputs
#define MOT_A1_PIN 2   // og 10
#define MOT_A2_PIN 4   // og 9
#define MOT_B1_PIN 18    // og 6
#define MOT_B2_PIN 19    // og 5

#define SLP_PIN 13

void notify()
{

    printPS3ButtonRawValues();
   //printJoystickRawValues();
   handleJoystickChanges();



   //---------------------- Battery events ---------------------
    //printBatteryStatus();

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

#define MAX_JS_RANGE 255

long joystickLeftX = 0;
long joystickLeftY = 0;
long joystickRightX = 0;
long joystickRightY = 0;

void handleJoystickChanges(){
    // Left Stick
   if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 5 ){
       Serial.print("Left stick:");

        long rawX = Ps3.data.analog.stick.lx;
        long rawY = Ps3.data.analog.stick.ly;

        joystickLeftX = map(rawX, -127, 127, -100, 100);
        joystickLeftY = map(rawY, 127, -127, -100, 100);

        Serial.print(" percentY="); Serial.print(joystickLeftY, DEC); Serial.print(" rawY="); Serial.print(rawY, DEC);
        Serial.println();

    }

    // Right Stick
   if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 5 ){
        Serial.print("Right stick:");

        long rawX = Ps3.data.analog.stick.rx;
        long rawY = Ps3.data.analog.stick.ry;

        joystickRightX = map(rawX, -127, 127, -100, 100);
        joystickRightY = map(rawY, 127, -127, -100, 100);

        Serial.print(" percentY="); Serial.print(joystickRightY, DEC); Serial.print(" rawY="); Serial.print(rawY, DEC);
        Serial.println();

   }
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
    if( battery != Ps3.data.status.battery ){
        battery = Ps3.data.status.battery;
        Serial.print("The controller battery is ");
        if( battery == ps3_status_battery_charging )      Serial.println("charging");
        else if( battery == ps3_status_battery_full )     Serial.println("FULL");
        else if( battery == ps3_status_battery_high )     Serial.println("HIGH");
        else if( battery == ps3_status_battery_low)       Serial.println("LOW");
        else if( battery == ps3_status_battery_dying )    Serial.println("DYING");
        else if( battery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
        else Serial.println("UNDEFINED");
    }
}

void onConnect(){
    Serial.println("Connected.");
}

void setup()
{
    Serial.println("Setup() started");
    Serial.begin(115200);

    setupMotors();

    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);

    Ps3.begin(PS3_BLACK_BLACK_1);

    Ps3.setPlayer(player);

    //-------------------- Player LEDs -------------------
    Serial.print("Setting LEDs to player "); Serial.println(player, DEC);
    
    Serial.println("Setup() ended");
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

    delay(2000);
}

/// Set the current on a motor channel using PWM and directional logic.
///
/// \param pwm    PWM duty cycle ranging from -255 full reverse to 255 full forward
/// \param IN1_PIN  pin number xIN1 for the given channel
/// \param IN2_PIN  pin number xIN2 for the given channel
void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

/// Set the current on both motors.
///
/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255
void set_motor_currents(int pwm_A, int pwm_B)
{
  set_motor_pwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  set_motor_pwm(pwm_B, MOT_B1_PIN, MOT_B2_PIN);

  // Print a status message to the console.
  Serial.print("Set motor A PWM = ");
  Serial.print(pwm_A);
  Serial.print(" motor B PWM = ");
  Serial.println(pwm_B);
}

/// Simple primitive for the motion sequence to set a speed and wait for an interval.
///
/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255
/// \param duration delay in milliseconds
void spin_and_wait(int pwm_A, int pwm_B, int duration)
{
  set_motor_currents(pwm_A, pwm_B);
  delay(duration);
}
