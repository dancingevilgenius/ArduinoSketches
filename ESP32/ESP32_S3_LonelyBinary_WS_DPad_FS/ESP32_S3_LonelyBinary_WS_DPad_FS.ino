#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h> 
#include <ArduinoJson.h>



// Network credentials Here
//const char* ssid     = "STDL5301";	    const char* password = "library30";	// Change this for your project
//const char* ssid     = "TheMandalorian";  const char* password = "6302201111";	// Change this for your project
const char* ssid     = "TheMandaloriKen"; const char* password = "asdf12346302201111";	// Change this for your project


NetworkServer server(80); 

// PSRAM buffer for index.html
char* htmlPage = nullptr;
size_t htmlSize = 0;



// Horizontal menu (fixed 4 items)
const char* horizontalMenu[] = { "SPEED", "TURNING", "PROPORTIONAL", "INTEGRAL" };
int horizontalIndex = 0;
const int horizontalCount = 4;

// Vertical lists for each horizontal menu
const char* verticalMenu_M1[] = { "50", "60", "70", "80", "90", "100" };
const char* verticalMenu_M2[] = { "50", "60", "70", "80", "90", "100" };
const char* verticalMenu_M3[] = { "50", "60", "70", "80", "90" };
const char* verticalMenu_M4[] = { "0.1", "0.2", "0.3", "0.4", "0.5" };

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

// ---------------------
//  8x8 web grid colors
// ---------------------
#define RED_OFFSET    0
#define GREEN_OFFSET  10
unsigned int gridColors[8][8];
int currentRow = 3;
int currentCol = 5;



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
  delay(2000);
  Serial.begin(115200);
  delay(2000);

  Serial.println("Webserver DPAD FS setup()");

  esp_log_level_set("*", ESP_LOG_NONE);  

  setupWebServer();

  setupGridColors();

}

void setupGridColors() {

  // Bottom red gradient (unchanged)
  gridColors[7][0] = RED_OFFSET + 0;
  gridColors[7][1] = RED_OFFSET + 1;
  gridColors[7][2] = RED_OFFSET + 2;
  gridColors[7][3] = RED_OFFSET + 3;
  gridColors[7][4] = RED_OFFSET + 4;
  gridColors[7][5] = RED_OFFSET + 5;
  gridColors[7][6] = RED_OFFSET + 6;
  gridColors[7][7] = RED_OFFSET + 7;

  // Only ONE green cell now
  gridColors[3][5] = GREEN_OFFSET + 7;

  // Track its starting position
  currentRow = 3;
  currentCol = 5;

  Serial.println("setupGridColors() completed. Initial display pattern");
}



//
// ------------------------------------------------------------
// NEW FUNCTION: setupWebServer()
// ------------------------------------------------------------
//
void setupWebServer() {

    // Check PSRAM
    if (!psramFound()) {
        Serial.println("PSRAM not found! Make sure it's enabled in board settings.");
    } else {
        Serial.println("PSRAM detected.");
    }
    delay(1000);

    // Mount LittleFS
    if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed. Did you upload index.html?");
    } else {
        Serial.println("LittleFS mounted.");
    }
    delay(1000);

    // Load index.html from LittleFS into PSRAM
    if (!loadIndexHtmlToPSRAM()) {
        Serial.println("Failed to load index.html into PSRAM.");
    }
    delay(1000);

    // WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    delay(1000);

    server.begin();
}



bool isFormatted = false;



void listAvailableFiles(){

  File root;
  File file;

  root = LittleFS.open("/");
  file = root.openNextFile();

  Serial.println("Start List of files on microcontroller:");
  while(file){
      Serial.print("FILE: ");
      Serial.println(file.name());
      file = root.openNextFile();
  }

  Serial.println("End listing files.\n");
}





void loop() {
  loopWebServer();
}

void loopWebServer(){

  WiFiClient client = server.available();
  if (!client) return;

  String request = "";
  unsigned long timeout = millis();

  while (client.connected() && millis() - timeout < 2000) {
    if (client.available()) {
      char c = client.read();
      request += c;
      if (request.endsWith("\r\n\r\n")) break;
    }
  }

  if (request.startsWith("GET / ") || request.startsWith("GET /index.html")) {
    serveHTML(client);
    client.stop();
    return;
  }

  if (request.startsWith("POST /controller")) {

    int clIndex = request.indexOf("Content-Length:");
    int contentLength = 0;
    if (clIndex != -1) {
      int start = clIndex + 15;
      int end = request.indexOf("\r\n", start);
      contentLength = request.substring(start, end).toInt();
    }

    String body = "";
    while (client.available() < contentLength) delay(1);
    while (client.available()) body += (char)client.read();

    //Serial.println("=== JSON BODY RECEIVED ===");
    //Serial.println(body);

    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error) {
      Serial.print("JSON parse failed: ");
      Serial.println(error.c_str());
    } else {

      // ----------------------------
      // HANDLE DIRECTION
      // ----------------------------
      if (doc.containsKey("direction")) {
        const char* direction = doc["direction"];
        Serial.print("Direction: ");
        Serial.println(direction);
        navigateMenu(direction);
      }

      // ----------------------------
      // HANDLE START / STOP ACTIONS
      // ----------------------------
      if (doc.containsKey("action")) {
        const char* action = doc["action"];
        Serial.print("Action: ");
        Serial.println(action);

        if (strcmp(action, "start") == 0) {
          Serial.println(">>> START triggered");
        }
        else if (strcmp(action, "stop") == 0) {
          Serial.println(">>> STOP triggered");
        }
      }

      // ----------------------------
      // HANDLE DROPDOWN MENU
      // ----------------------------
      if (doc.containsKey("menu")) {
        const char* menuValue = doc["menu"];
        Serial.print("Dropdown selected: ");
        Serial.println(menuValue);
      }

      // ----------------------------
      // FRONTEND PERIODIC GRID REQUEST
      // ----------------------------
      if (doc.containsKey("requestGrid")) {

          StaticJsonDocument<700> response;

          JsonArray grid = response.createNestedArray("grid");
          for (int r = 0; r < 8; r++) {
              JsonArray row = grid.createNestedArray();
              for (int c = 0; c < 8; c++) {
                  row.add(gridColors[r][c]);
              }
          }

          String out;
          serializeJson(response, out);

          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: application/json");
          client.println("Connection: close");
          client.println();
          client.println(out);
          return;
      }
    }

    // Build JSON response
    StaticJsonDocument<200> response;
    response["horiz"] = horizontalMenu[horizontalIndex];
    response["vert"]  = verticalMenus[horizontalIndex][verticalIndex];

    String out;
    serializeJson(response, out);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.println(out);
    client.stop();
    return;
  }

  client.println("HTTP/1.1 400 Bad Request");
  client.println("Connection: close");
  client.println();
  client.stop();

}

void handleDirectionParam(String direction){
    if (direction == "up") {
      //pixels.fill(0xFF00FF);
      Serial.println("up");
    }
    else if (direction == "down") {
      //pixels.fill(0xFF0000);
      Serial.println("down");
    }
    else if (direction == "left") {
      //pixels.fill(0x0000FF);
      Serial.println("left");
    }
    else if (direction == "right") {
      //pixels.fill(0x00FF00);
      Serial.println("right");
    }
    else if (direction == "center") {
      //pixels.fill(0x000);
      Serial.println("center");
    } else {
      //pixels.fill(0x000);
      Serial.println("unknown");
    }
    //pixels.show();
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
      //pixels.fill(0xFF00FF);
    }
    else if (direction == "down") {
      //pixels.fill(0xFF0000);
    }
    else if (direction == "left") {
      //pixels.fill(0x0000FF);
    }
    else if (direction == "right") {
      //pixels.fill(0x00FF00);
    }
    else if (direction == "center") {
      //pixels.fill(0x000);
    }
    //pixels.show();

  }

}


//
// ------------------------------------------------------------
// loadIndexHtmlToPSRAM()
// ------------------------------------------------------------
//
bool loadIndexHtmlToPSRAM() {
    File file = LittleFS.open("/index_dpad.html", "r");
    if (!file) {
        Serial.println("Failed to open /index_dpad.html from LittleFS");
        delay(1000);
        return false;
    }
    delay(1000);

    htmlSize = file.size();
    if (htmlSize == 0) {
        Serial.println("/index.html is empty");
        file.close();
        delay(1000);
        return false;
    }

    htmlPage = (char*)ps_malloc(htmlSize + 1);
    if (!htmlPage) {
        Serial.println("ps_malloc failed (no PSRAM?)");
        file.close();
        delay(1000);
        return false;
    }

    size_t readBytes = file.readBytes(htmlPage, htmlSize);
    file.close();

    if (readBytes != htmlSize) {
        Serial.println("Failed to read full index.html into PSRAM");
        free(htmlPage);
        htmlPage = nullptr;
        htmlSize = 0;
        delay(1000);
        return false;
    }

    htmlPage[htmlSize] = '\0';
    Serial.printf("Loaded /index.html into PSRAM (%u bytes)\n", (unsigned)htmlSize);
    delay(1000);
    return true;
}

//
// ------------------------------------------------------------
// serveHTML()
// ------------------------------------------------------------
//
void serveHTML(WiFiClient &client) {
    if (!htmlPage || htmlSize == 0) {
        client.println("HTTP/1.1 500 Internal Server Error");
        client.println("Content-Type: text/plain");
        client.println("Connection: close");
        client.println();
        client.println("index.html not loaded");
        Serial.println("serveHTML() fail. 500");
        return;
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.write(htmlPage, htmlSize);
    Serial.println("serveHTML() success. 200");
}

//
// ------------------------------------------------------------
// sendOK()
// ------------------------------------------------------------
//
void sendOK(WiFiClient &client, const char* msg) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println(msg);
}


void sendResponseHeader(WiFiClient client) {
    
    // Should not normally edit/remove these 4 lines
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();

}

