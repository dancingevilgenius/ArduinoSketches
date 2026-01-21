#include "TCS34725.h"
TCS34725 tcs;

void setup(void)
{
    Serial.begin(115200);

    Wire.begin();
    if (!tcs.attach(Wire)){
        Serial.println("ERROR: TCS34725 NOT FOUND !!!");
    } else {
        Serial.println("SUCCES: TCS34725 initialized.");
    }

    tcs.integrationTime(33); // ms
    tcs.gain(TCS34725::Gain::X01);

    // set LEDs...
}

void loop(void)
{
    if (tcs.available()) // if current measurement has done
    {
        TCS34725::Color color = tcs.color();
        Serial.print("Color Temp : "); Serial.println(tcs.colorTemperature());
        Serial.print("Lux        : "); Serial.println(tcs.lux());
        Serial.print("R          : "); Serial.println(color.r);
        Serial.print("G          : "); Serial.println(color.g);
        Serial.print("B          : "); Serial.println(color.b);
        delay(1000);
    }
}