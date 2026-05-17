// Webserver game controller DPad web page code uploaded to Adafruit Forums by:
// Carlos Garcia (dancingevilgenius) on May 16, 2026
// Original board tested: Adafruit QT PY Pico
// Original Espressif library version:  3.3.8
// Original Adafruit Neopixel version: 1.15.5

#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

// How many internal neopixels do we have? some boards have more than one!
#define NUMPIXELS        1

Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Network credentials Here
const char* ssid     = "STDL5301";	// Change this for your project
const char* password = "library30";	// Change this for your project

// Set web server port number to 80
NetworkServer server(80);

bool verbose = false; // Used to hide some of the less important web server connection properties.


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
  NetworkClient client = server.available(); // Wait for an incoming client
  
  if (client) {
    String request = "";
    
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        request += c;
        
        // End of client HTTP request header
        if (c == '\n' && request.endsWith("\r\n\r\n")) {
          
          // 1. Parse URL parameters  (e.g., /?direction=left)
          handleClientRequest(request);

          // 2. Send HTTP Response Header
          sendResponseHeader(client);
          
          // 3. Serve the Web Page
          sendWebPage(client);

          break; // Break out of the while loop
        }
      }
    }
    
    // Close the connection
    client.stop();

    // Small delay so that the loop isn't working every millisecond.
    delay(100);
  }
  
}

void sendResponseHeader(NetworkClient client) {
    
    // Should not normally edit/remove these 4 lines
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();

}

void sendWebPage(NetworkClient client){

  // Send your web page here.
  // In this case it is a simulated game controller DPad.
  clientDPad(client);
}

void clientDPad(NetworkClient client){

  // Inside your request handler, after client.println("HTTP/1.1 200 OK"); etc.
  client.println("<!DOCTYPE html>");
  client.println("<html>");
  client.println("<head>");
  client.println("<title>DPad</title>");
  client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  client.println("<style>");
  client.println("body { font-family: Arial, sans-serif; text-align: center; background: #111; color: #eee; }");
  client.println(".dpad-container { display: inline-block; margin-top: 40px; }");
  client.println(".row { display: flex; justify-content: center; }");
  client.println(".btn {");
  client.println("  width: 70px;");
  client.println("  height: 70px;");
  client.println("  margin: 5px;");
  client.println("  border-radius: 10px;");
  client.println("  border: none;");
  client.println("  background: #333;");
  client.println("  color: #fff;");
  client.println("  font-size: 24px;");
  client.println("  cursor: pointer;");
  client.println("}");
  client.println(".btn:active { background: #555; }");
  client.println("</style>");
  client.println("</head>");
  client.println("<body>");
  client.println("<h2>Web D-Pad</h2>");
  client.println("<div class=\"dpad-container\">");

  client.println("  <div class=\"row\">");
  client.println("    <form action=\"/up \" method=\"GET\">");
  client.println("      <button class=\"btn\" type=\"submit\" name=\"direction\" value=\"up\">&#9650;</button>");
  client.println("    </form>");
  client.println("  </div>");

  client.println("  <div class=\"row\">");
  client.println("    <form action=\"/left\" method=\"GET\">");
  client.println("      <button class=\"btn\" type=\"submit\" name=\"direction\" value=\"left\">&#9664;</button>");
  client.println("    </form>");
  client.println("    <form action=\"/center\" method=\"GET\">");
  client.println("      <button class=\"btn\" type=\"submit\" name=\"direction\" value=\"center\">OK</button>");
  client.println("    </form>");
  client.println("    <form action=\"/right\" method=\"GET\">");
  client.println("      <button class=\"btn\" type=\"submit\" name=\"direction\" value=\"right\">&#9654;</button>");
  client.println("    </form>");
  client.println("  </div>");

  client.println("  <div class=\"row\">");
  client.println("    <form action=\"/down\" method=\"GET\">");
  client.println("      <button class=\"btn\" type=\"submit\" name=\"direction\" value=\"down\">&#9660;</button>");
  client.println("    </form>");
  client.println("  </div>");

  client.println("</div>");
  client.println("</body>");
  client.println("</html>");
  client.println("");
}

String getParam(String request, String key) {
    int keyIndex = request.indexOf(key + "=");
    if (keyIndex == -1) return "";

    int start = keyIndex + key.length() + 1;
    int end = request.indexOf('&', start);
    if (end == -1) end = request.indexOf(' ', start);

    return request.substring(start, end);
}

void handleClientRequest(String request) {

  handleRequestParamDirection(request); // DPad sends form data as 'direction' param.
}

void handleRequestParamDirection(String request){
  String direction = getParam(request, "direction");

  String dirSet[] = {"up", "down", "left", "right", "center"};

  int setSize = 5;
  bool found = false;

  for (int i = 0 ; i < setSize ; i++) {
    if (direction == dirSet[i]) {
      found = true;
      break; // Exit loop early once match is found
    }
  }

  //if (direction.length() > 0) {
  if(found){
    Serial.print("Direction pressed: ");
    Serial.println(direction);

    if (direction == "up") {
      pixels.fill(0xFF00FF);
    }
    else if (direction == "down") {
      pixels.fill(0xFF0000);
    }
    else if (direction == "left") {
      pixels.fill(0x0000FF);
    }
    else if (direction == "right") {
      pixels.fill(0x00FF00);
    }
    else if (direction == "center") {
      pixels.fill(0x000);
    }
    pixels.show();

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
