#include <Button.h>

Button button1(D0); // 2 Connect your button between pin 2 and GND
Button button2(D1); // 3 Connect your button between pin 3 and GND
Button button3(D2); // 4 Connect your button between pin 4 and GND

void setup() {
	button1.begin();
	button2.begin();
	button3.begin();
	
	while (!Serial) { }; // for Leos
	Serial.begin(9600);
}

void loop() {
	if (button1.pressed())
		Serial.println("Button 1 pressed");
	
	if (button2.pressed())
		Serial.println("Button 2 pressed");

	if (button3.pressed())
		Serial.println("Button 3 pressed");

  /*
	if (button3.toggled()) {
		if (button3.read() == Button::PRESSED)
			Serial.println("Button 3 has been pressed");
		else
			Serial.println("Button 3 has been released");
	}
  */
 delay(50); // add a delay between scans   
}