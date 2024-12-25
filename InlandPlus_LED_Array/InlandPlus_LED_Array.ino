#define LED_PIN_1 3
#define LED_PIN_2 5
#define LED_PIN_3 6
#define LED_PIN_4 9
#define LED_PIN_5 10

#define LED_NUMBER 5
byte LEDPinArray[LED_NUMBER] = { LED_PIN_1,
                                 LED_PIN_2,
                                 LED_PIN_3,
                                 LED_PIN_4,
                                 LED_PIN_5 };
void setup()
{
  for (int i = 0; i < LED_NUMBER; i++) {
    pinMode(LEDPinArray[i], OUTPUT);
    digitalWrite(LEDPinArray[i], HIGH);
  }
}
void loop() {

if (digitalRead(BUTTON_PIN) == LOW) {
    digitalWrite(LED_PIN, HIGH);
  }  

}