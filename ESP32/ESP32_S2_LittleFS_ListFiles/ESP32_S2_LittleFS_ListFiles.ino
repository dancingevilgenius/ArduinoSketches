#include <Arduino.h>
#include <LittleFS.h>

File root;
File file;

void setup() {
  Serial.begin(112500);
  delay(1000);

  LittleFS.begin();
  Serial.println("Exiting setup()");
  delay(1000);

}

void loop() {


  root = LittleFS.open("/");
  file = root.openNextFile();

  Serial.println("Start List of files on microcontroller:");
  while(file){
      Serial.print("FILE: ");
      Serial.println(file.name());
      file = root.openNextFile();
  }

  Serial.println("End listing files.\n");

  delay(2000);

}
