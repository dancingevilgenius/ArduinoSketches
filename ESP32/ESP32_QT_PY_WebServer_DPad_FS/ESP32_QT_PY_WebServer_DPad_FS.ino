#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h> 
#include <Adafruit_NeoPixel.h>

// How many internal neopixels do we have? some boards have more than one!
#define NUMPIXELS        1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);


// Network credentials Here
const char* ssid     = "STDL5301";	// Change this for your project
const char* password = "library30";	// Change this for your project

NetworkServer server(80);

void setup() {
  Serial.begin(115200);

  delay(2000);

  boolean success=false;



  success = setupLittleFS();
  if(!success){
    return;
  }


  setupWifiConnection();

  setupNeopixel();

}

void setupWifiConnection(){
 WiFi.begin(ssid,password);
  int count=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Attempting to connect. count:" + count++);
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();

}

boolean setupLittleFS(){

  bool success = false;
  // 1. Start LittleFS
  if(!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed");
    success = false;
  } else {
    success = true;
  }

  return success;
}

void loop() {
  NetworkClient client = server.available();

  

  if (client) {
    while (client.connected()) {
      if (client.available()) {
        String req = client.readStringUntil('\r');
        client.flush();

        // 2. Simple route for "/" or "/index.html"
        if (req.indexOf("GET /") != -1) {
        //if(req.startsWith("\n") && req.endsWith("\r\n\r\n")) {
          File file = LittleFS.open("/index.html", "r");
          if (file) {
            // 3. Send HTTP Headers
            sendResponseHeader(client);

            // 4. Stream the file content
            while (file.available()) {
              client.write(file.read());
            }
            file.close();
          } else {
            client.println("HTTP/1.1 404 Not Found");
          }
        }
        break;
      }
    }
    client.stop();
  }
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

  // Show Green to start
  pixels.fill(0x00FF00);
  pixels.show();
}


void sendResponseHeader(NetworkClient client) {
    
    // Should not normally edit/remove these 4 lines
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();

}

