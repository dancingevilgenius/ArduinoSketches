#define LED 2 // works with ESP32 DEV board, Acebott ESP32-Max

void setup() {
    pinMode(LED, OUTPUT);
}

void loop() {
    digitalWrite(LED, HIGH);
    delay(700);
    digitalWrite(LED, LOW);
    delay(500);
}