// ------------------------------------------------------------
// Project: D-Pad loaded from HTML
// Backend: ESP32 AsyncWebServer + AsyncWebSocket + LittleFS
// Frontend: index_dpad.html (WebSocket grid)
// Status: Merged baseline (HTTP D-Pad + WS grid, no patterns)
// ------------------------------------------------------------

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <map>

// ------------------------------------------------------------
// WiFi Credential Rotation (REORDERED)
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

// ------------------------------------------------------------
// Web Server + WebSocket
// ------------------------------------------------------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Store User-Agent strings per WebSocket client ID
std::map<uint32_t, String> clientUserAgents;

// ------------------------------------------------------------
// 8×8 Grid (uint16_t) for WebSocket + demo animation
// ------------------------------------------------------------
uint16_t colorGrid[8][8];

// simple right-to-left green pixel animation
int currentRow = 3;
int currentCol = 5;

// ------------------------------------------------------------
// PSRAM HTML Buffer
// ------------------------------------------------------------
char* htmlBuffer = nullptr;
size_t htmlSize = 0;

// ------------------------------------------------------------
// Animation / Control State (no pattern logic)
// ------------------------------------------------------------
bool animationRunning = true;

unsigned long lastAnimationTime = 0;
unsigned long lastClientCleanupTime = 0;

unsigned long INTERVAL_ANIMATION = 100;           // ms, default 10 FPS
unsigned long INTERVAL_CLIENT_CLEANUP = 5000;     // ms, default 5 seconds

float brightness = 1.0f;
enum SendMode { MODE_FULL, MODE_BITMASK };
SendMode sendMode = MODE_FULL;

// ------------------------------------------------------------
// D-Pad menu state (from original backend)
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
// Device Parsing Helpers (for client list)
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

  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      if (colorGrid[r][c] != 0) {
        bits |= (uint64_t)1 << (r * 8 + c);
      }
    }
  }

  ws.binaryAll((uint8_t*)&bits, sizeof(bits));
}

// ------------------------------------------------------------
// Demo animation: single green pixel moving right→left
// ------------------------------------------------------------
void initGrid() {
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      colorGrid[r][c] = 0;
    }
  }

  currentRow = 3;
  currentCol = 5;
  uint16_t base = (uint16_t)(32 * brightness);
  colorGrid[currentRow][currentCol] = base;
}

void updateGridAnimation() {
  colorGrid[currentRow][currentCol] = 0;

  int newCol = currentCol - 1;
  if (newCol < 0) newCol = 7;

  currentCol = newCol;

  uint16_t base = (uint16_t)(32 * brightness);
  colorGrid[currentRow][currentCol] = base;
}

// ------------------------------------------------------------
// Command parsing (no pattern logic)
// ------------------------------------------------------------
void handleCommand(const String& msg, AsyncWebSocketClient* client) {
  if (msg.startsWith("UA:")) {
    clientUserAgents[client->id()] = msg.substring(3);
    Serial.printf("UA received for client %u\n", client->id());
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
  else if (msg.startsWith("BRI:")) {
    float b = msg.substring(4).toFloat();
    brightness = constrain(b, 0.0f, 1.0f);
  }
  else if (msg.startsWith("MODE:")) {
    String m = msg.substring(5);
    sendMode = (m == "BIT") ? MODE_BITMASK : MODE_FULL;
  }
}

// ------------------------------------------------------------
// Multi-client viewer list + cleanup
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
// Setup Web Server (root + /controller + WebSocket)
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

  // Serve HTML
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

  // D-Pad /controller endpoint (JSON in, JSON out, no grid)
  server.on(
    "/controller",
    HTTP_POST,
    [](AsyncWebServerRequest *request) { /* response sent in body handler */ },
    NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String body;
      body.reserve(total);
      for (size_t i = 0; i < len; i++) body += (char)data[i];

      StaticJsonDocument<300> doc;
      DeserializationError err = deserializeJson(doc, body);
      if (err) {
        request->send(400, "application/json", "{\"error\":\"bad json\"}");
        return;
      }

      // direction / action handling (same as original, but no grid)
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
// Extracted animation loop
// ------------------------------------------------------------
void loopAnimation() {
  unsigned long now = millis();

  if (animationRunning && now - lastAnimationTime >= INTERVAL_ANIMATION) {
    lastAnimationTime = now;

    updateGridAnimation();

    if (ws.count() > 0) {
      if (sendMode == MODE_FULL) sendFullGrid();
      else sendBitGrid();
    }
  }
}

// ------------------------------------------------------------
// Extracted client cleanup loop
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
// setup()
// ------------------------------------------------------------
void setup() {
  delay(2000);
  Serial.begin(115200);
  delay(2000);

  Serial.println("Merged D-Pad + WebSocket backend setup()");

  connectToWiFi();
  initGrid();
  setupWebServer();
}

// ------------------------------------------------------------
// loop()
// ------------------------------------------------------------
void loop() {
  loopAnimation();
  loopClientCleanup();
}
