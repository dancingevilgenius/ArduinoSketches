//#define LED 2 // works with ESP32 DEV board, Acebott ESP32-Max


#define BUILTIN_LED 15  // ESP32-S2 Mini


#define ONE_SEC 1000
void setup() {
    Serial.begin(115200);
    pinMode(BUILTIN_LED, OUTPUT);
}

void blink(int count, int interval, int wait_ms){
    for(int i=0 ; i<count ; i++){
        digitalWrite(BUILTIN_LED, HIGH);
        delay(interval);
        digitalWrite(BUILTIN_LED, LOW);
        delay(interval);
    }

    delay(wait_ms);
}

void loop() {
    Serial.println("blink cycle");

    int wait_ms = ONE_SEC;
    blink(1, 1000, wait_ms);
    blink(3, 250, wait_ms);
    blink(5, 200, wait_ms);

}