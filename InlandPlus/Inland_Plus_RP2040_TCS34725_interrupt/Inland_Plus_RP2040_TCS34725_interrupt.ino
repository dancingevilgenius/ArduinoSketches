#include <Wire.h>
#include <Adafruit_TCS34725.h>


/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
                                            TCS34725_INTEGRATIONTIME_199MS, //TCS34725_INTEGRATIONTIME_614MS,
                                            TCS34725_GAIN_1X);
const int interruptPin = 3; // 2 for some boards. Using 3 on Inland Plus
volatile boolean state = false;


//Interrupt Service Routine
void isr() 
{
  state = true;
}


/* tcs.getRawData() does a delay(Integration_Time) after the sensor readout.
We don't need to wait for the next integration cycle because we receive an interrupt when the integration cycle is complete*/
void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}


void setup() {

  Serial.begin(115200);
  delay(2000);
  Serial.println("setup() started");
  pinMode(interruptPin, INPUT_PULLUP); //TCS interrupt output is Active-LOW and Open-Drain
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr, FALLING);

  Wire.begin();
  
  if (tcs.begin(TCS34725_ADDRESS, &Wire)) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  
  // Set persistence filter to generate an interrupt for every RGB Cycle, regardless of the integration limits
  tcs.write8(TCS34725_PERS, TCS34725_PERS_NONE); 
  tcs.setInterrupt(true);
  
  Serial.flush();
  Serial.println("setup() ended.");
}


void loop() {
  if (state) {
    uint16_t r, g, b, c, colorTemp, lux;
    getRawData_noDelay(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    lux = tcs.calculateLux(r, g, b);
    
    //printRawValues(r,g,b,c, colorTemp,lux);
    bool isLineHit = isLineDetected(r,g,b,c, colorTemp,lux);
    if(isLineHit){
      Serial.println("Line!");
    } else {
      Serial.println("-");
    }

    Serial.flush();

    tcs.clearInterrupt();
    state = false;
  } else {
    //Serial.println("sensor not ready.");
    //delay(1000);
  }
}



bool isLineDetected(uint16_t r, uint16_t g, uint16_t b,uint16_t c, uint16_t colorTemp, uint16_t lux){
    // Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
    // Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
    // Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    // Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    // Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    // Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    // Serial.println(" ");
    return(lux < 5000);
}

void printRawValues(uint16_t r, uint16_t g, uint16_t b,uint16_t c, uint16_t colorTemp, uint16_t lux){
    Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
    Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    Serial.println(" ");
}
