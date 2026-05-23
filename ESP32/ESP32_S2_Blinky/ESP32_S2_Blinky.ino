
#define PIN_LED 15

void setup()
{
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(115200);
}

void loop()
{
    Serial.println("Hello, world!");
    digitalWrite(PIN_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);       // wait for a second

    
    digitalWrite(PIN_LED, LOW);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second
    digitalWrite(PIN_LED, HIGH);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second
    digitalWrite(PIN_LED, LOW);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second
}
