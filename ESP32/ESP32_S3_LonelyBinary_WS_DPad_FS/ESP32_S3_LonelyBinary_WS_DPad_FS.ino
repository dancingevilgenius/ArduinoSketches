#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// ------------------------------------------------------------
// WiFi Credential Rotation
// ------------------------------------------------------------
struct WifiCredential {
  const char* ssid;
  const char* password;
};

WifiCredential wifiList[3] = {
  { "TheMandaloriKen","asdf12346302201111" },
  { "STDL5301",       "library30" },
  { "TheMandalorian", "6302201111" }
};

// Pending message to send to frontend
String pendingMessage = "";
String pendingSeverity = "info";

NetworkServer server(80);

// PSRAM buffer for HTML
char* htmlPage = nullptr;
size_t htmlSize = 0;

// Sensor loop timing
#define SENSOR_INTERVAL_TIME 300
long lastSensorUpdateTime = 0;

// Horizontal menu
const char* horizontalMenu[] = { "SPEED", "TURNING", "PROPORTIONAL", "INTEGRAL" };
int horizontalIndex = 0;
const int horizontalCount = 4;

// Vertical lists
const char* verticalMenu_M1[] = { "50", "60", "70", "80", "90", "100" };
const char* verticalMenu_M2[] = { "50", "60", "70", "80", "90", "100" };
const char* verticalMenu_M3[] = { "50", "60", "70", "80", "90" };
const char* verticalMenu_M4[] = { "0.1", "0.2", "0.3", "0.4", "0.5" };

const char** verticalMenus[] = {
  verticalMenu_M1,
  verticalMenu_M2,
  verticalMenu_M3,
  verticalMenu_M4
};

int verticalCounts[] = {
  sizeof(verticalMenu_M1) / sizeof(verticalMenu_M1[0]),
  sizeof(verticalMenu_M2) / sizeof(verticalMenu_M2[0]),
  sizeof(verticalMenu_M3) / sizeof(verticalMenu_M3[0]),
  sizeof(verticalMenu_M4) / sizeof(verticalMenu_M4[0])
};

int verticalIndex = 0;

// 8x8 grid
#define RED_OFFSET    0
#define GREEN_OFFSET  10
unsigned int gridColors[8][8];
int currentRow = 3;
int currentCol = 5;


// ------------------------------------------------------------
// loadIndexHtmlToPSRAM() — filename-aware loader
// ------------------------------------------------------------
bool loadIndexHtmlToPSRAM(const char* filename) {

    File file = LittleFS.open(filename, "r");
    if (!file) {
        Serial.print("Failed to open ");
        Serial.println(filename);
        return false;
    }

    htmlSize = file.size();
    if (htmlSize == 0) {
        Serial.print(filename);
        Serial.println(" is empty");
        file.close();
        return false;
    }

    htmlPage = (char*)ps_malloc(htmlSize + 1);
    if (!htmlPage) {
        Serial.println("ps_malloc failed (no PSRAM?)");
        file.close();
        return false;
    }

    size_t readBytes = file.readBytes(htmlPage, htmlSize);
    file.close();

    if (readBytes != htmlSize) {
        Serial.print("Failed to read full ");
        Serial.print(filename);
        Serial.println(" into PSRAM");
        free(htmlPage);
        htmlPage = nullptr;
        htmlSize = 0;
        return false;
    }

    htmlPage[htmlSize] = '\0';

    Serial.print("Loaded ");
    Serial.print(filename);
    Serial.print(" into PSRAM (");
    Serial.print((unsigned)htmlSize);
    Serial.println(" bytes)");

    return true;
}


// ------------------------------------------------------------
// connectToWiFi() — rotates through 3 SSID/password pairs
// ------------------------------------------------------------
bool connectToWiFi() {

    Serial.println("Starting WiFi credential rotation...");

    for (int i = 0; i < 3; i++) {

        Serial.print("Trying SSID: ");
        Serial.println(wifiList[i].ssid);

        WiFi.begin(wifiList[i].ssid, wifiList[i].password);

        int failCount = 0;

        while (WiFi.status() != WL_CONNECTED && failCount < 5) {
            delay(500);
            Serial.print(".");
            failCount++;
        }

        if (WiFi.status() == WL_CONNECTED) {

            Serial.println("\nConnected!");

            // Queue message for frontend
            pendingMessage = String("Connected to ") + wifiList[i].ssid;
            pendingSeverity = "success";

            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());

            return true;
        }

        Serial.println("\nFailed to connect. Moving to next SSID...");
    }

    pendingMessage = "Failed to connect to any WiFi network";
    pendingSeverity = "error";

    Serial.println("ERROR: Could not connect to ANY WiFi network.");
    return false;
}


// ------------------------------------------------------------
// setupWebServer()
// ------------------------------------------------------------
void setupWebServer() {

    if (!psramFound()) {
        Serial.println("PSRAM not found!");
    } else {
        Serial.println("PSRAM detected.");
    }

    if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed.");
    } else {
        Serial.println("LittleFS mounted.");
    }

    // Try index_dpad.html first
    if (!loadIndexHtmlToPSRAM("/index_dpad.html")) {
        Serial.println("Trying fallback: /index.html");

        if (!loadIndexHtmlToPSRAM("/index.html")) {
            Serial.println("ERROR: No valid HTML file found.");
        }
    }

    // WiFi connection
    connectToWiFi();

    server.begin();
}


// ------------------------------------------------------------
// setupGridColors()
// ------------------------------------------------------------
void setupGridColors() {

  gridColors[7][0] = RED_OFFSET + 0;
  gridColors[7][1] = RED_OFFSET + 1;
  gridColors[7][2] = RED_OFFSET + 2;
  gridColors[7][3] = RED_OFFSET + 3;
  gridColors[7][4] = RED_OFFSET + 4;
  gridColors[7][5] = RED_OFFSET + 5;
  gridColors[7][6] = RED_OFFSET + 6;
  gridColors[7][7] = RED_OFFSET + 7;

  gridColors[3][5] = GREEN_OFFSET + 7;

  currentRow = 3;
  currentCol = 5;

  Serial.println("setupGridColors() completed.");
}


// ------------------------------------------------------------
// navigateMenu()
// ------------------------------------------------------------
void navigateMenu(const String& direction) {

  if (direction == "left") {
    horizontalIndex--;
    if (horizontalIndex < 0)
      horizontalIndex = horizontalCount - 1;
    verticalIndex = 0;
  }

  else if (direction == "right") {
    horizontalIndex++;
    if (horizontalIndex >= horizontalCount)
      horizontalIndex = 0;
    verticalIndex = 0;
  }

  else if (direction == "up") {
    verticalIndex--;
    if (verticalIndex < 0)
      verticalIndex = verticalCounts[horizontalIndex] - 1;
  }

  else if (direction == "down") {
    verticalIndex++;
    if (verticalIndex >= verticalCounts[horizontalIndex])
      verticalIndex = 0;
  }

  else if (direction == "center") {
    Serial.println("Center pressed — select/confirm");
  }

  Serial.print("Menu: ");
  Serial.print(horizontalMenu[horizontalIndex]);
  Serial.print(" | Item: ");
  Serial.println(verticalMenus[horizontalIndex][verticalIndex]);
}


// ------------------------------------------------------------
// loop8x8Sensors()
// ------------------------------------------------------------
void loop8x8Sensors() {

  long now = millis();
  long dt = now - lastSensorUpdateTime;

  if (dt < SENSOR_INTERVAL_TIME) return;

  unsigned int value = gridColors[currentRow][currentCol];
  gridColors[currentRow][currentCol] = 0;

  int newCol = currentCol - 1;
  if (newCol < 0) newCol = 7;

  gridColors[currentRow][newCol] = value;
  currentCol = newCol;

  lastSensorUpdateTime = now;
}


// ------------------------------------------------------------
// loopSensors()
// ------------------------------------------------------------
void loopSensors() {
  loop8x8Sensors();
}


// ------------------------------------------------------------
// serveHTML()
// ------------------------------------------------------------
void serveHTML(WiFiClient &client) {

    if (!htmlPage || htmlSize == 0) {
        client.println("HTTP/1.1 500 Internal Server Error");
        client.println("Content-Type: text/plain");
        client.println("Connection: close");
        client.println();
        client.println("HTML not loaded");
        return;
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.write(htmlPage, htmlSize);
}


// ------------------------------------------------------------
// loopWebServer()
// ------------------------------------------------------------
void loopWebServer() {

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

    StaticJsonDocument<300> doc;
    if (!deserializeJson(doc, body)) {

      if (doc.containsKey("direction")) {
        navigateMenu(doc["direction"]);
      }

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

    // Build response
    StaticJsonDocument<200> response;
    response["horiz"] = horizontalMenu[horizontalIndex];
    response["vert"]  = verticalMenus[horizontalIndex][verticalIndex];

    // Add pending message if any
    if (pendingMessage.length() > 0) {
        response["message"] = pendingMessage;
        response["severity"] = pendingSeverity;
        pendingMessage = "";
    }

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


// ------------------------------------------------------------
// setup()
// ------------------------------------------------------------
void setup() {
  delay(2000);
  Serial.begin(115200);
  delay(2000);

  Serial.println("Webserver DPAD FS setup()");

  esp_log_level_set("*", ESP_LOG_NONE);

  setupWebServer();
  setupGridColors();
}


// ------------------------------------------------------------
// loop()
// ------------------------------------------------------------
void loop() {
  loopSensors();
  loopWebServer();
}
