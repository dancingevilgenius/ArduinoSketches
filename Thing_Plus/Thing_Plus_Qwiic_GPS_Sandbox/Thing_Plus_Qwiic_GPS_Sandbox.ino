#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example - Modified by Carlos Feb 23,2024");

  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART1 port to output UBX only (turn off NMEA noise)  

  //This will pipe all NMEA sentences to the serial port so we can see them
  myGNSS.setNMEAOutputPort(Serial);
}

void loop()
{
  //myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

    if (myGNSS.getPVT() == true)
    {
      int32_t latitude = myGNSS.getLatitude();
      //Serial.print(F("Lat: "));
      Serial.print("Lat: ");
      Serial.print(latitude);

      int32_t longitude = myGNSS.getLongitude();
      //Serial.print(F(" Long: "));
      Serial.print(" Long: ");
      Serial.print(longitude);
      //Serial.print(F(" (degrees * 10^-7)"));
      //Serial.print(F(" (degrees * 10^-7)"));

      /*
      int32_t altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
      Serial.print(F(" Alt: "));
      Serial.print(altitude);
      Serial.print(F(" (mm)"));
      */

      Serial.println();
    } else {
        Serial.print(F("PVT is false. No reading"));
        delay (1000);
    }


  delay(250); //Don't pound too hard on the I2C bus
}
