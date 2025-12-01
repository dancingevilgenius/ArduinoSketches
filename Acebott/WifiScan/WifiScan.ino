/*
When a network is found in the environment, 
the number and name of all nearby networks will be obtained and displayed in the serial port, 
and the blue indicator will light up.
*/
#include "WiFi.h"
void setup()
{
    Serial.begin(115200);
    pinMode(02,OUTPUT);
    //set WiFi to station mode and disconnect from an AP if it was previously connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    Serial.println("Setup done");
}
void loop()
{
    Serial.println("scan start");
    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        digitalWrite(2, HIGH);//the blue indicator lights up
        for (int i = 0; i < n; ++i) {
            //print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
            delay(10);
        }
    }
    Serial.println("");
    // wait a bit before scanning again
    delay(5000);
}