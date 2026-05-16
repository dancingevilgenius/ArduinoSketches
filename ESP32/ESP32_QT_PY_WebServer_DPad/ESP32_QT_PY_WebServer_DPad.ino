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
            //clientLEDControls(client);
            clientDPad(client);
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }


      }

      // Parse client requests/commands here
      handleClientRequest(currentLine);

    }


    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

void clientDPad(NetworkClient client){

  // Inside your request handler, after client.println("HTTP/1.1 200 OK"); etc.
  client.println("<!DOCTYPE html>");
  client.println("<html>");
  client.println("<head>");
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
  client.println("    <form action=\"/up\" method=\"GET\">");
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
        //Serial.println("up");
    }
    else if (direction == "down") {
        //Serial.println("down");
    }
    else if (direction == "left") {
        //Serial.println("left");
    }
    else if (direction == "right") {
        //Serial.println("right");
    }
    else if (direction == "center") {
        Serial.println("center");
    }
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
