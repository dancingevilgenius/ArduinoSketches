// --------------------------------------
// I2C Scanner example using Wire1
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>

TwoWire Wire1(SDA,SCL); //Intialize Wire1 class

HardwareSerial Serial6(D0, D1); // RX:D0:GREEN, TX:D1:WHITE works

void setup()
{
  Wire1.begin();

  Serial6.begin(115200); // 115200
  while (!Serial6);             // Leonardo: wait for serial monitor
  Serial6.println("\nI2C Scanner");
}


void loop()
{
  byte error, address;
  int nDevices;

  Serial6.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
      Serial6.print("I2C device found at address 0x");
      if (address<16)
        Serial6.print("0");
      Serial6.print(address,HEX);
      Serial6.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial6.print("Unknown error at address 0x");
      if (address<16)
        Serial6.print("0");
      Serial6.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial6.println("No I2C devices found\n");
  else
    Serial6.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
