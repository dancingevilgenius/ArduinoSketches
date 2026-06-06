#include "Wire.h"

// QT PY Pico:Wire1
#define WIRE_I2C Wire1

void setup() {
  Serial.begin(115200);
  WIRE_I2C.setPins(SDA1, SCL1);
  WIRE_I2C.begin(); 
}

void loop() {
  byte error, address;
  int nDevices = 0;

  delay(5000);

  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0x7f; address++) {
    WIRE_I2C.beginTransmission(address);
    error = WIRE_I2C.endTransmission();
    if (error == 0) {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if (error != 2) {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  }
}
