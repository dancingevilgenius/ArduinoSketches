/*
  Bas.Button

  This example code is in the public domain.
*/

#include<Bas.Button.h>

const int buttonPin0 = D0;  // the number of the pushbutton pin. ov:2
const int buttonPin1 = D1;  // the number of the pushbutton pin. ov:2
const int buttonPin2 = D2;  // the number of the pushbutton pin. ov:2
const unsigned long debounceDelay = 15; // Amount of milliseconds to debounce

Bas::Button button1{ buttonPin0, debounceDelay, Bas::Button::LogLevel::none };    // Instantiate a Button object with the correct pin and debouncedelay, and set the log level to none to prevent it from writing event logs to Serial.
Bas::Button button2{ buttonPin1, debounceDelay, Bas::Button::LogLevel::none };    // Instantiate a Button object with the correct pin and debouncedelay, and set the log level to none to prevent it from writing event logs to Serial.
Bas::Button button3{ buttonPin2, debounceDelay, Bas::Button::LogLevel::none };    // Instantiate a Button object with the correct pin and debouncedelay, and set the log level to none to prevent it from writing event logs to Serial.

/// <summary>
/// This method will be called when the pushbutton is pushed.
/// </summary>
void onButtonPressed()
{
    Serial.println("Button pushed down.");
}

void setup() {
    //Initialize serial and wait for port to open:
    Serial.begin(9600);
    while (!Serial);  // wait for serial port to connect. Needed for native USB port only

    button1.begin(onButtonPressed);
    button2.begin(onButtonPressed);
    button3.begin(onButtonPressed);
}

void loop() {
    // read the state of the switch into a local variable:
    button1.update();    // The button needs to be updated on each loop to be responsive.
    button2.update();    // The button needs to be updated on each loop to be responsive.
    button3.update();    // The button needs to be updated on each loop to be responsive.

    //Serial.print("Button state: ");

    if (button1.isPressed())
    {
        Serial.println("B1 pressed.");
    }
    else
    {
        //Serial.println("not pressed.");
    }

    if (button2.isPressed())
    {
        Serial.println("B2 pressed.");
    }
    else
    {
        //Serial.println("not pressed.");
    }

    if (button3.isPressed())
    {
        Serial.println("B3 pressed.");
        
    }
    else
    {
        //Serial.println("not pressed.");
    }
    //delay(500);

}