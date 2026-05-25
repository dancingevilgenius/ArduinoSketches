#include <Arduino.h>
#include <LittleFS.h>


void setup() {
  Serial.begin(112500);
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  delay(2000);
  Serial.println("Start List of files on microcontroller:");
  while(file){
      Serial.print("FILE: ");
      Serial.println(file.name());
      file = root.openNextFile();
  }

  Serial.println("End listing files.");

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("loop .");
  delay(5000);

}
