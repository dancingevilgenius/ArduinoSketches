//  Mini Sumo
//  Hardware:
//    DFRobot TOF 8x8 Matrix
//    Adafruit 13x9 LED Matrix
//
//  Other Features:
//    Send 8x8 matrix data to web page
//    Load from index_dpad.html file saved on PSRAM.

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

#define TOF_8x8_NUM_ROWS 8
#define TOF_8x8_NUM_COLS 8
#define X_OFFSET 2
#define D_OPP_MIN 1
#define D_OPP_MAX 30
#define D_EDGE6_MAX 12
#define D_EDGE7_MAX 8

uint8_t matrix[TOF_8x8_NUM_ROWS][TOF_8x8_NUM_COLS] = {
  {0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0},
  {0,0,0,20,20,0,0,0},
  {0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0},
  {10,10,10,10,10,10,10,10},
  {10,10,10,8,10,10,10,10}
};

// 8×8 WebSocket grid
uint16_t colorGrid[8][8];

// ToF Sensor
DFRobot_MatrixLidar_I2C tof(0x33, &Wire1);
uint16_t lidarGrid[64];

#define INVALID_VAL 4000
#define RING_SIZE_MM 770
#define ROBOT_SIZE_MM 100
#define MAX_DIST 570

#define SDA_PIN 22
#define SCL_PIN 23

// ------------------------------------------------------------
// Web Server + WebSocket
// ------------------------------------------------------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

std::map<uint32_t, String> clientUserAgents;

// ------------------------------------------------------------
// PSRAM HTML Buffer
// ------------------------------------------------------------
char* htmlBuffer = nullptr;
size_t htmlSize = 0;

// ------------------------------------------------------------
// Animation / Control State
// ------------------------------------------------------------
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
  while(tof.begin() != 0){
    Serial.println("DFR MatrixLidar not found");
    delay(500);
  }
  Serial.println("DFR MatrixLidar found.");

  int count=0;
  Serial.println("DFR MatrixLidar init starting...");
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

  clearLEDMatrix();
  clearLEDMatrix();
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
// ToF → LED Matrix → WebSocket Grid
// ------------------------------------------------------------
void loopMiniSumoOpponent() {
  uint16_t oppColor      = ledmatrix.color565(0, 150, 0);
  uint16_t edgeColor     = ledmatrix.color565(150, 0, 0);
  uint16_t edgeWarnColor = ledmatrix.color565(180, 180, 0);

  tof.getAllData(lidarGrid);

  for(uint8_t y = 0; y < 8; y++){
    for(uint8_t x = 0; x < 8; x++){
      int d_mm = lidarGrid[y * 8 + x];

      if (d_mm == INVALID_VAL || d_mm > MAX_DIST) {
        ledmatrix.drawPixel(x+X_OFFSET, y, 0);
        colorGrid[y][x] = 0;
        continue;
      }

      if (y == 3 || y == 4) {
        if (d_mm < 500) {
          ledmatrix.drawPixel(x+X_OFFSET, y, oppColor);
          colorGrid[y][x] = 8888;
        }
      }
      else if (y == 6) {
        if (d_mm > 200) {
          ledmatrix.drawPixel(x+X_OFFSET, y, edgeWarnColor);
          colorGrid[y][x] = 999999999;
        }
      }
      else if (y == 7) {
        if (d_mm > 170) {
          ledmatrix.drawPixel(x+X_OFFSET, y, edgeColor);
          colorGrid[y][x] = 999999999;
        }
      }
      else {
        ledmatrix.drawPixel(x+X_OFFSET, y, 0);
        colorGrid[y][x] = 0;
      }
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
// LED Matrix Helpers
// ------------------------------------------------------------
void clearLEDMatrix() {
  ledmatrix.fill(0);
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
    if (u.indexOf("sm-s92") >= 0) return "Samsung Galaxy S24 Ultra (Android)";
    if (u.indexOf("sm-s91") >= 0) return "Samsung Galaxy S24 (Android)";
    if (u.indexOf("pixel") >= 0) return "Google Pixel (Android)";
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
