//  Mini Sumo Backend — TRUE RGB VERSION
//  Sends real 0xRRGGBB colors to the front-end

#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_MatrixLidar.h"
#include <Adafruit_IS31FL3741.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <map>

// ------------------------------------------------------------
// WiFi Credential Rotation
// ------------------------------------------------------------
struct WifiCredential {
  const char* ssid;
  const char* password;
};

WifiCredential wifiList[3] = {
  { "TheMandaloriKen", "asdf12346302201111" },
  { "TheMandalorian",  "6302201111" },
  { "STDL5301",        "library30" }
};

String pendingMessage = "";
String pendingSeverity = "info";

Adafruit_IS31FL3741_QT ledmatrix;

#define X_OFFSET 2
#define INVALID_VAL 4000
#define MAX_DIST 570

uint32_t colorGrid[8][8];

DFRobot_MatrixLidar_I2C tof(0x33, &Wire1);
uint16_t lidarGrid[64];

// ------------------------------------------------------------
// Web Server + WebSocket
// ------------------------------------------------------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

std::map<uint32_t, String> clientUserAgents;

char* htmlBuffer = nullptr;
size_t htmlSize = 0;

bool animationRunning = true;

unsigned long lastAnimationTime = 0;
unsigned long lastClientCleanupTime = 0;

unsigned long INTERVAL_ANIMATION = 100;       // default 10 FPS
unsigned long INTERVAL_CLIENT_CLEANUP = 5000; // 5 seconds

enum SendMode { MODE_FULL, MODE_BITMASK };
SendMode sendMode = MODE_FULL;

// ------------------------------------------------------------
// D-Pad menu state
// ------------------------------------------------------------
const char* horizontalMenu[] = { "SPEED", "TURNING", "PROPORTIONAL", "INTEGRAL" };
int horizontalIndex = 0;
const int horizontalCount = 4;

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

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  delay(2000);

  Serial.println("MiniSumo QT PY Pico LedMatrix and DFR8x8");

  connectToWiFi();
  setupI2C();
  setupLedMatrix();
  setupDFR8x8();
  setupWebServer();
}

// ------------------------------------------------------------
// Setup Web Server
// ------------------------------------------------------------
void setupWebServer() {
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed.");
  } else {
    Serial.println("LittleFS mounted.");
  }

  if (!loadHTMLToPSRAM("/index_dpad.html")) {
    Serial.println("FATAL: Could not load /index_dpad.html");
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!htmlBuffer || htmlSize == 0) {
      request->send(500, "text/plain", "HTML not loaded");
      return;
    }
    AsyncWebServerResponse *response =
      request->beginResponse(200, "text/html",
                             (const uint8_t*)htmlBuffer, htmlSize);
    response->addHeader("Cache-Control", "no-cache");
    request->send(response);
  });

  // D-Pad JSON endpoint
  server.on(
    "/controller",
    HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String body;
      body.reserve(total);
      for (size_t i = 0; i < len; i++) body += (char)data[i];

      StaticJsonDocument<300> doc;
      if (deserializeJson(doc, body)) {
        request->send(400, "application/json", "{\"error\":\"bad json\"}");
        return;
      }

      if (doc.containsKey("direction")) {
        String direction = doc["direction"];

        if (direction == "left") {
          horizontalIndex--;
          if (horizontalIndex < 0) horizontalIndex = horizontalCount - 1;
          verticalIndex = 0;
        }
        else if (direction == "right") {
          horizontalIndex++;
          if (horizontalIndex >= horizontalCount) horizontalIndex = 0;
          verticalIndex = 0;
        }
        else if (direction == "up") {
          verticalIndex--;
          if (verticalIndex < 0) verticalIndex = verticalCounts[horizontalIndex] - 1;
        }
        else if (direction == "down") {
          verticalIndex++;
          if (verticalIndex >= verticalCounts[horizontalIndex]) verticalIndex = 0;
        }
      }

      if (doc.containsKey("action")) {
        String action = doc["action"];
        if (action == "start") animationRunning = true;
        if (action == "stop")  animationRunning = false;
      }

      StaticJsonDocument<256> response;
      response["horiz"] = horizontalMenu[horizontalIndex];
      response["vert"]  = verticalMenus[horizontalIndex][verticalIndex];

      if (pendingMessage.length() > 0) {
        response["message"]  = pendingMessage;
        response["severity"] = pendingSeverity;
        pendingMessage = "";
      }

      String out;
      serializeJson(response, out);
      request->send(200, "application/json", out);
    }
  );

  // WebSocket
  ws.onEvent([](AsyncWebSocket *server,
                AsyncWebSocketClient *client,
                AwsEventType type,
                void *arg,
                uint8_t *data,
                size_t len) {

    if (type == WS_EVT_CONNECT) {
      Serial.printf("WS: Client %u connected | IP: %s\n",
                    client->id(),
                    client->remoteIP().toString().c_str());


    }

    if (type == WS_EVT_DISCONNECT) {
      Serial.printf("WS: Client %u disconnected\n", client->id());
    }

    if (type == WS_EVT_DATA) {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;

      if (info->opcode == WS_TEXT) {
        String msg;
        for (size_t i = 0; i < len; i++) msg += (char)data[i];

        if (msg == "PING") {
          client->text("PONG");
          return;
        }

        handleCommand(msg, client);
      }
    }
  });

  server.addHandler(&ws);
  server.begin();
}

// ------------------------------------------------------------
// Command parsing
// ------------------------------------------------------------
void handleCommand(const String& msg, AsyncWebSocketClient* client) {
  if (msg.startsWith("UA:")) {
    clientUserAgents[client->id()] = msg.substring(3);
    return;
  }

  if (msg.startsWith("RUN:")) {
    animationRunning = msg.substring(4).toInt();
  }
  else if (msg.startsWith("FPS:")) {
    int fps = msg.substring(4).toInt();
    fps = constrain(fps, 1, 60);
    INTERVAL_ANIMATION = 1000UL / fps;
  }
  else if (msg.startsWith("MODE:")) {
    String m = msg.substring(5);
    sendMode = (m == "BIT") ? MODE_BITMASK : MODE_FULL;
  }
  else if (msg.startsWith("SNACK:")) {
      // Format: SNACK:type:message
      int p1 = msg.indexOf(':', 6);
      if (p1 > 0) {
          String type = msg.substring(6, p1);
          String text = msg.substring(p1 + 1);

          // Forward to all clients
          String payload = "SNACK:" + type + ":" + text;
          ws.textAll(payload);
      }
  }
  else if (msg == "HEALTHCHECK") {
    Serial.println("HEALTHCHECK command received");
    healthCheckWifi();
    healthCheckI2C();
    healthCheckLedMatrix();
    healthCheckDFR8x8();
    healthCheckWebServer();
  } 
}

// ------------------------------------------------------------
// Load HTML file from LittleFS → PSRAM
// ------------------------------------------------------------
bool loadHTMLToPSRAM(const char* filename) {
  if (!LittleFS.exists(filename)) {
    Serial.printf("ERROR: File %s not found\n", filename);
    return false;
  }

  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.println("ERROR: Failed to open HTML file");
    return false;
  }

  htmlSize = file.size();
  htmlBuffer = (char*)ps_malloc(htmlSize + 1);

  if (!htmlBuffer) {
    Serial.println("ERROR: PSRAM allocation failed");
    file.close();
    return false;
  }

  file.readBytes(htmlBuffer, htmlSize);
  htmlBuffer[htmlSize] = '\0';
  file.close();

  Serial.printf("Loaded %s (%u bytes)\n", filename, (unsigned)htmlSize);
  return true;
}

// ------------------------------------------------------------
// WiFi rotation connect
// ------------------------------------------------------------
bool connectToWiFi() {
  Serial.println("Starting WiFi credential rotation...");

  for (int i = 0; i < 3; i++) {
    Serial.print("Trying SSID: ");
    Serial.println(wifiList[i].ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiList[i].ssid, wifiList[i].password);

    int failCount = 0;
    while (WiFi.status() != WL_CONNECTED && failCount < 20) {
      delay(500);
      Serial.print(".");
      failCount++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected!");
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
// I2C + Sensor Setup
// ------------------------------------------------------------
void setupI2C() {
  Wire1.begin();
}

void setupDFR8x8() {
  int count=0;
  while(tof.begin() != 0 ){
    Serial.print(".");
    count++;
    if(count > 10){
      Serial.println("DFR MatrixLidar not found. Exiting setupDFR8x8()");
      return;
    }
    delay(500);
  }
  Serial.println("DFR MatrixLidar found.");

  Serial.println("DFR MatrixLidar init starting...");
  count = 0;
  while(tof.setRangingMode(eMatrix_8X8) != 0){
    Serial.print(count);
    Serial.print(" ");
    ++count;
    if(count > 10){
      Serial.println("DFR MatrixLidar not initialized.");
      return;
    }
    delay(250);
  }

  Serial.println("DFR MatrixLidar initialized!");
}

void setupLedMatrix() {
  if (! ledmatrix.begin(IS3741_ADDR_DEFAULT, &Wire1)) {
    Serial.println("LED 13x9 Matrix not found");
    return;
  }

  Serial.println("LED 13x9 Matrix found!");

  ledmatrix.setLEDscaling(0xAA);
  ledmatrix.setGlobalCurrent(0xCC);
  ledmatrix.enable(true);

  ledmatrix.fill(0);
  delay(2000);
}

// ------------------------------------------------------------
// Main Loop
// ------------------------------------------------------------
void loop() {
  loopMiniSumoOpponent();
  loopAnimation();
  loopClientCleanup();
  delay(80);
}

// ------------------------------------------------------------
// Animation Loop
// ------------------------------------------------------------
void loopAnimation() {
  unsigned long now = millis();

  if (animationRunning && now - lastAnimationTime >= INTERVAL_ANIMATION) {
    lastAnimationTime = now;

    if (ws.count() > 0) {
      if (sendMode == MODE_FULL)
        sendFullGrid();
      else
        sendBitGrid();
    }
  }
}

// ------------------------------------------------------------
// MINI-SUMO COLOR LOGIC — TRUE RGB
// ------------------------------------------------------------
void loopMiniSumoOpponent() {

  tof.getAllData(lidarGrid);

  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 8; x++) {

      int d_mm = lidarGrid[y * 8 + x];

      // Invalid or too far → black
      if (d_mm == INVALID_VAL || d_mm > MAX_DIST) {
        ledmatrix.drawPixel(x + X_OFFSET, y, 0);
        colorGrid[y][x] = 0;
        continue;
      }

      // -----------------------------
      // Rows 0–4: Opponent detection (bright green)
      // -----------------------------
      if (y <= 4) {
        if (d_mm > 0) {
          ledmatrix.drawPixel(x + X_OFFSET, y, ledmatrix.color565(0,255,0));
          colorGrid[y][x] = 0x00FF00;
        }
        continue;
      }

      // -----------------------------
      // Row 6: Edge warning (bright yellow)
      // -----------------------------
      if (y == 6) {
        if (d_mm > 0) {
          ledmatrix.drawPixel(x + X_OFFSET, y, ledmatrix.color565(255,255,0));
          colorGrid[y][x] = 0xFFFF00;
        }
        continue;
      }

      // -----------------------------
      // Row 7: Edge danger (bright red)
      // -----------------------------
      if (y == 7) {
        if (d_mm > 0) {
          ledmatrix.drawPixel(x + X_OFFSET, y, ledmatrix.color565(255,0,0));
          colorGrid[y][x] = 0xFF0000;
        }
        continue;
      }

      // Row 5 or anything else → black
      ledmatrix.drawPixel(x + X_OFFSET, y, 0);
      colorGrid[y][x] = 0;
    }
  }
}

// ------------------------------------------------------------
// Send full 8×8 grid (128 bytes)
// ------------------------------------------------------------
void sendFullGrid() {
  ws.binaryAll((uint8_t*)colorGrid, sizeof(colorGrid));
}

// ------------------------------------------------------------
// Send compact 64-bit bitmask
// ------------------------------------------------------------
void sendBitGrid() {
  uint64_t bits = 0;

  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      if (colorGrid[y][x] != 0) {
        bits |= (uint64_t)1 << (y * 8 + x);
      }
    }
  }

  ws.binaryAll((uint8_t*)&bits, sizeof(bits));
}


// ------------------------------------------------------------
// HEALTH CHECKS — each sends a snackbar via WebSocket
// ------------------------------------------------------------

void healthCheckWifi() {
    if (WiFi.status() == WL_CONNECTED) {
        ws.textAll("SNACK:success:WiFi OK (" + WiFi.localIP().toString() + ")");
    } else {
        ws.textAll("SNACK:error:WiFi NOT connected");
    }
}

void healthCheckI2C() {
    // Simple check: try a transmission on Wire1
    Wire1.beginTransmission(0x00);
    uint8_t err = Wire1.endTransmission();

    if (err == 0 || err == 2) {
        ws.textAll("SNACK:success:I2C Bus OK");
    } else {
        ws.textAll("SNACK:error:I2C Bus Error");
    }
}

void healthCheckLedMatrix() {
    // Check if LED driver responded during init
    if (ledmatrix.begin(IS3741_ADDR_DEFAULT, &Wire1)) {
        ws.textAll("SNACK:success:LED Matrix OK");
    } else {
        ws.textAll("SNACK:error:LED Matrix NOT responding");
    }
}

void healthCheckDFR8x8() {
    uint16_t tempGrid[64];

    if ( tof.begin() == 0 && tof.getAllData(tempGrid) == 0) {
        ws.textAll("SNACK:success:DFR 8×8 Lidar OK");
    } else {
        ws.textAll("SNACK:error:DFR 8×8 Lidar NOT responding");
    }
}

void healthCheckWebServer() {
    // Check if our web page has been loaded into htmlBuffer
    if(htmlBuffer != nullptr){
      ws.textAll("SNACK:success:Web Server OK");
    } else {
      ws.textAll("SNACK:error:Web Server web page not found");
    }
}



// ------------------------------------------------------------
// Client Cleanup
// ------------------------------------------------------------
void loopClientCleanup() {
  unsigned long now = millis();

  if (now - lastClientCleanupTime > INTERVAL_CLIENT_CLEANUP) {
    lastClientCleanupTime = now;
    printClientList();
    ws.cleanupClients();
  }
}

// ------------------------------------------------------------
// Client List Debug
// ------------------------------------------------------------
void printClientList() {
  String hostIP = WiFi.localIP().toString();
  int32_t rssi = WiFi.RSSI();

  Serial.printf("---- WebSocket Clients (Host: %s | RSSI: %d dBm) ----\n",
                hostIP.c_str(), rssi);

  for (AsyncWebSocketClient& c : ws.getClients()) {
    AsyncWebSocketClient* client = &c;

    uint32_t cid = client->id();
    IPAddress ip = client->remoteIP();

    String ua = clientUserAgents.count(cid) ? clientUserAgents[cid] : "Unknown";
    String device = parseDeviceName(ua);
    String browser = parseBrowser(ua);

    if (client->status() == WS_CONNECTED) {
      Serial.printf(
        "Client %u: CONNECTED | IP: %s | Device: %s | Browser: %s\n",
        cid,
        ip.toString().c_str(),
        device.c_str(),
        browser.c_str()
      );
    } else {
      Serial.printf(
        "Client %u: NOT CONNECTED (closing) | IP: %s | Device: %s | Browser: %s\n",
        cid,
        ip.toString().c_str(),
        device.c_str(),
        browser.c_str()
      );
      client->close();
    }
  }

  Serial.printf("Active client count: %u\n", ws.count());
  Serial.println("-------------------------------------------");
}

// ------------------------------------------------------------
// Device Parsing Helpers
// ------------------------------------------------------------
String parseDeviceName(const String& ua) {
  String u = ua;
  u.toLowerCase();

  if (u.indexOf("iphone") >= 0) return "iPhone (iOS)";
  if (u.indexOf("ipad") >= 0) return "iPad (iOS)";
  if (u.indexOf("mac os") >= 0 || u.indexOf("macintosh") >= 0) return "Mac";

  if (u.indexOf("android") >= 0) {
    if (u.indexOf("sm-s92") >= 0) return "Samsung Galaxy S24 Ultra";
    if (u.indexOf("sm-s91") >= 0) return "Samsung Galaxy S24";
    if (u.indexOf("pixel") >= 0) return "Google Pixel";
    return "Android Device";
  }

  if (u.indexOf("windows nt") >= 0) return "Windows PC";
  if (u.indexOf("linux") >= 0) return "Linux PC";

  return "Unknown Device";
}

String parseBrowser(const String& ua) {
  String u = ua;
  u.toLowerCase();

  if (u.indexOf("chrome/") >= 0) return "Chrome";
  if (u.indexOf("safari/") >= 0 && u.indexOf("chrome") < 0) return "Safari";
  if (u.indexOf("firefox/") >= 0) return "Firefox";
  if (u.indexOf("edg/") >= 0) return "Edge";

  return "Unknown Browser";
}



