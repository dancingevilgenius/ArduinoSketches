// Load Wi-Fi library
#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

// How many internal neopixels do we have? some boards have more than one!
#define NUMPIXELS        1

Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Network credentials Here
const char* ssid     = "STDL5301";
const char* password = "library30";

// Set web server port number to 80
NetworkServer server(80);

// Variable to store the HTTP request
String header;

//variables to store the current LED states
String statePin16 = "off";
String statePin17 = "off";


void setup() {
  Serial.begin(115200);


  delay(2000); // Fixes problem that displays ONLY firmware debugging info.

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

  setupNeopixel();
}


void loop() {
  NetworkClient client = server.accept();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client

    while (client.connected()) {
      // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            client.println("<style>html { font-family: monospace; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: yellowgreen; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 32px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: gray;}</style></head>");

            client.println("<body><h1>ESP32 QT PY Web Server</h1>");
            client.println("<p>Control Built-in Neopixel</p>");

            if (statePin16 == "off") {
              client.println("<p><a href=\"/16/on\"><button class=\"button\">BLUE ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/16/off\"><button class=\"button button2\">BLUE OFF</button></a></p>");
            }
            if (statePin17 == "off") {
              client.println("<p><a href=\"/17/on\"><button class=\"button\">RED ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/17/off\"><button class=\"button button2\">RED OFF</button></a></p>");
            }
            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();

            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

          // turns the GPIOs on and off
          if (currentLine.endsWith("GET /16/on")) {
            statePin16 = "on";
            pixels.fill(0x0000FF);
            pixels.show();

          } else if (currentLine.endsWith("GET /16/off")) {
            statePin16 = "off";
            pixels.fill(0x000000);
            pixels.show();
          }
          
          if (currentLine.endsWith("GET /17/on")) {
            statePin17 = "on";
            pixels.fill(0xFF0000);
            pixels.show();
          } else if (currentLine.endsWith("GET /17/off")) {
            statePin17 = "off";
            pixels.fill(0x000000);
            pixels.show();
          }




      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    //client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
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
  pixels.fill(0xFF00FF);
  pixels.show();


}
