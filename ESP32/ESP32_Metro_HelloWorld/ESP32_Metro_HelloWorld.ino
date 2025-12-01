// Make sure to press the reset button after uploading code from the Arduino IDE to the ESP32-S3!


int led = LED_BUILTIN;

void setup() {
  // Some boards work best if we also make a serial connection
  Serial.begin(115200);

  // set LED to be an output pin
  pinMode(led, OUTPUT);
  Serial.println("What is thy bidding my master?");
}

void loop() {
  
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                // wait for a half second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
  Serial.println("I still serve.");
}