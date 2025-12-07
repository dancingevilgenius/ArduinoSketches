// Define the control inputs
#define MOT_A1_PIN 2   // og 10
#define MOT_A2_PIN 4   // og 9
#define MOT_B1_PIN 17    // og 6
#define MOT_B2_PIN 19    // og 5

#define SLP_PIN 13

void setup(void)
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

  // Initialize the serial UART at 9600 baud
  Serial.begin(115200);
}

void loop(void)
{
  // Generate a fixed motion sequence to demonstrate the motor modes.

  //forwardRampUp();

  //forwardFullSpeed();

  //reverseFullSpeed();

  //reverseRampUp();

  //forwardFullSpeed();

  //reverseFullSpeed();


  demo();

  // Stop. wait for 2 seconds
  spin_and_wait(0,0,2000);
}

void forwardFullSpeed(){
  // Full speed forward.
  spin_and_wait(255,255,2000);
}


void reverseFullSpeed(){
  // Full speed reverse.
  spin_and_wait(-255,-255,2000);
}


void forwardRampUp(){
  // Ramp speed up.
  for (int i = 0; i < 11; i++) {
    spin_and_wait(25*i, 25*i, 500);
  }

}


void reverseRampUp() {
  // Ramp speed into full reverse.
  for (int i = 0; i < 21 ; i++) {
    spin_and_wait(255 - 25*i, 255 - 25*i, 500);
  }
}





void demo(){
  // Full speed, forward, turn, reverse, and turn for a two-wheeled base.
  Serial.println("forward");
  spin_and_wait(255, 255, 2000);  // forward

  Serial.println("both stop");
  spin_and_wait(0, 0, 3000);

  Serial.println("turn R");
  spin_and_wait(-255, 255, 2000); // turn

  Serial.println("both stop");
  spin_and_wait(0, 0, 3000);

  Serial.println("reverse");
  spin_and_wait(-255, -255, 2000); // reverse

  Serial.println("both stop");
  spin_and_wait(0, 0, 2000);

  Serial.println("turn L");
  spin_and_wait(255, -255, 2000); // turn

  Serial.println("both stop");
  spin_and_wait(0, 0, 3000);

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