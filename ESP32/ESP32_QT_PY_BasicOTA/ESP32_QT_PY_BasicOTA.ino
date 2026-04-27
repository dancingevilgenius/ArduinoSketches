#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>

// How many internal neopixels do we have? some boards have more than one!
#define NUMPIXELS        1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);


#ifndef STASSID
#define STASSID "STDL8167"
#define STAPSK "library62"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 2040
  // ArduinoOTA.setPort(2040);

  // Hostname defaults to pico-[ChipID]
  // ArduinoOTA.setHostname("mypico");

  // No authentication by default
  //ArduinoOTA.setPassword("password");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);

    setupNeopixel();
  
  });



  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


}


void setupNeopixel(){
#if defined(NEOPIXEL_POWER)
  // If this board has a power control pin, we must set it to output and high
  // in order to enable the NeoPixels. We put this in an #if defined so it can
  // be reused for other boards without compilation errors
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(20); // not so bright

  Serial.println("Finished setupNeopixel()");
}

#define ESP32_OLED_LED_PIN 8

void loop() {
  ArduinoOTA.handle();

  // Test code
  // pinMode(ESP32_OLED_LED_PIN, OUTPUT);

  // Serial.println("start blinking");
  // digitalWrite(ESP32_OLED_LED_PIN, HIGH);
  // delay(250);
  // digitalWrite(ESP32_OLED_LED_PIN, LOW);
  // delay(250);
  // digitalWrite(ESP32_OLED_LED_PIN, HIGH);
  // delay(250);
  // digitalWrite(ESP32_OLED_LED_PIN, LOW);
  // delay(250);
  // digitalWrite(ESP32_OLED_LED_PIN, HIGH);
  // delay(250);
  // digitalWrite(ESP32_OLED_LED_PIN, LOW);
  // delay(250);
  // digitalWrite(ESP32_OLED_LED_PIN, HIGH);
  // delay(250);
  // digitalWrite(ESP32_OLED_LED_PIN, LOW);
  // delay(250);

  // delay(3000);

  // set color to red
  pixels.fill(0xFF0000);
  pixels.show(); 
  delay(500); // wait half a second

  // turn off
  pixels.fill(0x000000);
  pixels.show();
  delay(500); // wait half a second

}
