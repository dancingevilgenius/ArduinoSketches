#include "EspEasyGPIOInterrupt.h"

#define PIN_BUTTON GPIO_NUM_7

void gpioInt1Task() {
  Serial.println("gpioInt1");
  delay(100);
}

EspEasyGPIOInterrupt gpioInt1;

void setup() {
  Serial.begin(115200);

  // Set Input Mode(INPUT or INPUT_PULLDOWN or INPUT_PULLUP)
  pinMode(PIN_BUTTON, INPUT);

  // Int setting
  //   task : void function()
  //   gpio : 0 - 39(ESP32)
  //   mode : RISING or FALLING or CHANGE
  //   core : APP_CPU_NUM or PRO_CPU_NUM
  gpioInt1.begin(gpioInt1Task, PIN_BUTTON, FALLING);


  delay(1000);
  Serial.println("GPIOInterrupt test");

}

void loop() {
}
