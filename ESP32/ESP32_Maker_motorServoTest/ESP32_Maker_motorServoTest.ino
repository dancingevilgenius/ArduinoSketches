#include <Arduino.h>

#include <ESP32Servo.h>


#define PIN_SERVO_32 32
#define PIN_SERVO_33 33
#define PIN_SERVO_25 34
#define PIN_SERVO_26 35

Servo servo_32;
Servo servo_33;
Servo servo_25;
Servo servo_26;

void setup() {
  servo_32.attach(PIN_SERVO_32);
  servo_33.attach(PIN_SERVO_33);
  servo_25.attach(PIN_SERVO_25);
  servo_26.attach(PIN_SERVO_26);
  

}

void loop() {
  analogWrite(27, 255);   // M1
  analogWrite(13, 0);
  analogWrite(4, 255);    // M2
  analogWrite(2, 0);
  analogWrite(17, 255);   // M3
  analogWrite(12, 0);
  analogWrite(15, 255);   // M4
  analogWrite(14, 0);
  servo_32.write(0);
  servo_33.write(0);
  servo_25.write(0);
  servo_26.write(0);
  delay(1000);
  analogWrite(13, 255);   // M1
  analogWrite(27, 0);
  analogWrite(2, 255);    // M2
  analogWrite(4, 0);
  analogWrite(12, 255);   // M3
  analogWrite(17, 0);
  analogWrite(14, 255);   // M4
  analogWrite(15, 0);
  servo_32.write(90);
  servo_33.write(90);
  servo_25.write(90);
  servo_26.write(90);
  delay(1000);
  servo_32.write(180);
  servo_33.write(180);
  servo_25.write(180);
  servo_26.write(180);
  delay(1000);
}