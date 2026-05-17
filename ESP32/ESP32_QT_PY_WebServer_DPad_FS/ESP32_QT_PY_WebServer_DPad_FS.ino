#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h> 
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>

// How many internal neopixels do we have? some boards have more than one!
#define NUMPIXELS        1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);


// Network credentials Here
const char* ssid     = "STDL5301";	// Change this for your project
const char* password = "library30";	// Change this for your project

NetworkServer server(80);

File htmlFile;


// Horizontal menu (fixed 4 items)
const char* horizontalMenu[] = { "M1", "M2", "M3", "M4" };
int horizontalIndex = 0;
const int horizontalCount = 4;

// Vertical lists for each horizontal menu
const char* verticalMenu_M1[] = { "A1", "A2", "A3" };
const char* verticalMenu_M2[] = { "B1", "B2", "B3", "B4" };
const char* verticalMenu_M3[] = { "C1", "C2" };
const char* verticalMenu_M4[] = { "D1", "D2", "D3", "D4", "D5" };

// Pointer array to vertical menus
const char** verticalMenus[] = {
  verticalMenu_M1,
  verticalMenu_M2,
  verticalMenu_M3,
  verticalMenu_M4
};

// Length of each vertical list
int verticalCounts[] = {
  sizeof(verticalMenu_M1) / sizeof(verticalMenu_M1[0]),
  sizeof(verticalMenu_M2) / sizeof(verticalMenu_M2[0]),
  sizeof(verticalMenu_M3) / sizeof(verticalMenu_M3[0]),
  sizeof(verticalMenu_M4) / sizeof(verticalMenu_M4[0])
};

int verticalIndex = 0;

// ----------------------------
// NAVIGATION FUNCTION
// ----------------------------
void navigateMenu(const String& direction) {

  if (direction == "left") {
    horizontalIndex--;
    if (horizontalIndex < 0)
      horizontalIndex = horizontalCount - 1;  // wrap
    verticalIndex = 0; // reset vertical position
  }

  else if (direction == "right") {
    horizontalIndex++;
    if (horizontalIndex >= horizontalCount)
      horizontalIndex = 0;  // wrap
    verticalIndex = 0; // reset vertical position
  }

  else if (direction == "up") {
    verticalIndex--;
    if (verticalIndex < 0)
      verticalIndex = verticalCounts[horizontalIndex] - 1; // wrap
  }

  else if (direction == "down") {
    verticalIndex++;
    if (verticalIndex >= verticalCounts[horizontalIndex])
      verticalIndex = 0; // wrap
  }

  else if (direction == "center") {
    Serial.println("Center pressed — select/confirm");
  }

  // Debug output
  Serial.print("Menu: ");
  Serial.print(horizontalMenu[horizontalIndex]);
  Serial.print(" | Item: ");
  Serial.println(verticalMenus[horizontalIndex][verticalIndex]);
}

void setup() {
  Serial.begin(115200);

  delay(2000);


  setupLittleFS();

  setupWifiConnection();

  setupNeopixel();

}

void setupWifiConnection(){
  
 WiFi.begin(ssid,password);
  int count=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Attempting to connect. count:" + count++);
    if(count > 15){
      Serial.println("Too many wifi connection failed attempts. Exiting program");
      exit(3);
    }
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  server.begin();

}

boolean setupLittleFS(){

  // 1. Start LittleFS
  if(!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed. exiting program");
    exit(1);
  }

  htmlFile = LittleFS.open("/index.html", "r");
  if(!htmlFile){
    Serial.println("Failed to open index.html  exiting program.");
    exit(2);
  }  

}



void serveFile(WiFiClient &client, const char* path) {
  File file = LittleFS.open(path, "r");
  if (!file) {
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("File not found");
    return;
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();

  while (file.available()) {
    client.write(file.read());
  }
  file.close();
}

void loop() {
  NetworkClient client = server.available();
  if (!client) return;

  String request = "";
  unsigned long timeout = millis();

  // Read headers
  while (client.connected() && millis() - timeout < 2000) {
    if (client.available()) {
      char c = client.read();
      request += c;
      if (request.endsWith("\r\n\r\n")) break;
    }
  }

  // Serve index.html
  if (request.startsWith("GET / ") || request.startsWith("GET /index.html")) {
    serveFile(client, "/index.html");
    client.stop();
    return;
  }

  // Handle POST /controller
  if (request.startsWith("POST /controller")) {

      // Extract Content-Length
      int clIndex = request.indexOf("Content-Length:");
      int contentLength = 0;
      if (clIndex != -1) {
        int start = clIndex + 15;
        int end = request.indexOf("\r\n", start);
        contentLength = request.substring(start, end).toInt();
      }

      // Read POST body
      String body = "";
      while (client.available() < contentLength) delay(1);
      while (client.available()) body += (char)client.read();

      Serial.println("=== JSON BODY RECEIVED ===");
      Serial.println(body);

    // Parse JSON using ArduinoJson
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error) {
        Serial.print("JSON parse failed: ");
        Serial.println(error.c_str());
    } else {
        const char* direction = doc["direction"];
        Serial.print("Parsed direction: ");
        Serial.println(direction);
        handleDirectionParam(direction);
        navigateMenu(direction);
        //Serial.print("codename:");
        //Serial.println(doc["codename"]);
    }

    // Respond
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("OK");

    client.stop();
    return;
  }

  // Unknown request
  client.println("HTTP/1.1 400 Bad Request");
  client.println("Connection: close");
  client.println();
  client.stop();
}

void handleDirectionParam(String direction){
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
    } else {
      pixels.fill(0x000);
    }
    pixels.show();
}

void sendWebpage(NetworkClient client,  File htmlFile){
    while (htmlFile.available()) {
      client.write(htmlFile.read());
    }
    htmlFile.close();
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

