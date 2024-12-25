#include <xmotion.h>

void setup() {
//xmotion.BlinkDelay(100); //Make Blink Both Leds 100 millisecs.
}

void loop() {
  Serial.println("start xm_motor_example");
xmotion.Forward(100,1000); // %100 Speed, both motor forward 100mS.
xmotion.StopMotors(1000);  // 100ms Stop Both Motors
xmotion.Backward(70,2500); // %70 Speed, both motor backward 250mS.
xmotion.Right0(51,1000);  // %51 Speed, 1 second Right 0 Turn.
xmotion.Left0(20,3500);   // %20 Speed, 3.5 second Left 0 Turn.
xmotion.ArcTurn(20,60,250); // 250 millisecond %20 Speed Left, %60 Speed Right Motor
xmotion.MotorControl(-150,190); //Timeless -150/255 Left, 190/255 Right Speed
delay(100); //Delay for last MotorControl statement
  Serial.println("exit xm_motor_example\n\n");
}
