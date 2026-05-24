//#define LED 2 // works with ESP32 DEV board, Acebott ESP32-Max
#define LED 15  // ESP32-S2 Mini
void setup() {
    Serial.begin(115200);
    pinMode(LED, OUTPUT);
}

void blink(int count, int interval){
    for(int i=0 ; i<count ; i++){
        digitalWrite(LED, HIGH);
        delay(interval);
        digitalWrite(LED, LOW);
        delay(interval);
    }
}

void loop() {
    Serial.println("blink cycle");

    blink(1, 1000);
    blink(3, 250);
    delay(1000);
    blink(5, 200);
    delay(1000);

}