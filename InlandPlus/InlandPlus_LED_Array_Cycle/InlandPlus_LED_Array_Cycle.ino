#define LED_PIN_1 3
#define LED_PIN_2 5
#define LED_PIN_3 6
#define LED_PIN_4 9
#define LED_PIN_5 10
int index = 0;

#define BUTTON_PIN 7

#define LED_NUMBER 3
byte LEDPinArray[LED_NUMBER] = { LED_PIN_1
                                 ,LED_PIN_2
                                 ,LED_PIN_3
                                 //,LED_PIN_4
                                 //,LED_PIN_5
                                 };
void setup()
{
  for (int i = 0; i < LED_NUMBER; i++) {
    pinMode(LEDPinArray[i], OUTPUT);
    digitalWrite(LEDPinArray[i], LOW);
  }

  Serial.begin(115200);
  Serial.println("\n Buttons initialized. starting index:" + index);
}
void loop() {

  delay(250);
  if (digitalRead(BUTTON_PIN) == LOW) {
      // Turn off old
      digitalWrite(LEDPinArray[index], LOW);
      index = index + 1;
      if(index > LED_NUMBER-1 ){
        index = 0;
      }
      Serial.println("Button index on:" + index);
      digitalWrite(LEDPinArray[index], HIGH);

  } 

}  

