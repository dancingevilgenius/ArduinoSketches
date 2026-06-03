#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <map>

// ------------------------------------------------------------
// WiFi Credentials
// ------------------------------------------------------------
const char* ssid = "TheMandaloriKen";
const char* password = "asdf12346302201111";

// ------------------------------------------------------------
// Web Server + WebSocket
// ------------------------------------------------------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Store User-Agent strings per WebSocket client ID
std::map<uint32_t, String> clientUserAgents;

// ------------------------------------------------------------
// 8×8 Grid (uint16_t)
// ------------------------------------------------------------
uint16_t colorGrid[8][8];

// ------------------------------------------------------------
// PSRAM HTML Buffer
// ------------------------------------------------------------
char* htmlBuffer = nullptr;
size_t htmlSize = 0;

// ------------------------------------------------------------
// Animation / Control State
// ------------------------------------------------------------
bool animationRunning = true;
unsigned long lastSend = 0;
unsigned long frameInterval = 100;  // ms, default 10 FPS
uint8_t patternIndex = 0;
float brightness = 1.0f;
enum SendMode { MODE_FULL, MODE_BITMASK };
SendMode sendMode = MODE_FULL;

// ------------------------------------------------------------
// WiFi Event Handler
// ------------------------------------------------------------
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {

    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("[WiFi] STA Started");
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WiFi] Connected to AP");
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("\n[WiFi] Connection Established!");
      Serial.print("[WiFi] IP Address: ");
      Serial.println(WiFi.localIP());
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("\n[WiFi] Connection Lost! Attempting to reconnect...");
      WiFi.reconnect();
      break;

    default:
      Serial.printf("[WiFi] Event: %d\n", event);
      break;
  }
}

// ------------------------------------------------------------
// WiFi Setup (non-blocking)
// ------------------------------------------------------------
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(WiFiEvent);

  Serial.printf("[WiFi] Connecting to %s...\n", ssid);
  WiFi.begin(ssid, password);
}

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
uint16_t applyBrightness(uint16_t v) {
    return (uint16_t)(v * brightness);
}

// ------------------------------------------------------------
// Load HTML file from LittleFS → PSRAM
// ------------------------------------------------------------
bool serveHTML(const char* filename) {
    if (!LittleFS.exists(filename)) {
        Serial.printf("ERROR: File %s not found\n", filename);
        return false;
    }

    File file = LittleFS.open(filename, "r");
    if (!file) {
        Serial.printf("ERROR: Could not open %s\n", filename);
        return false;
    }

    htmlSize = file.size();
    htmlBuffer = (char*)ps_malloc(htmlSize + 1);

    if (!htmlBuffer) {
        Serial.println("ERROR: Failed to allocate PSRAM for HTML");
        file.close();
        return false;
    }

    file.readBytes(htmlBuffer, htmlSize);
    htmlBuffer[htmlSize] = '\0';
    file.close();

    Serial.printf("Loaded %s into PSRAM (%u bytes)\n", filename, (unsigned)htmlSize);
    return true;
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
            int index = r * 8 + c;
            if (colorGrid[r][c] != 0) {
                bits |= (uint64_t)1 << index;
            }
        }
    }

    ws.binaryAll((uint8_t*)&bits, sizeof(bits));
}

// ------------------------------------------------------------
// Pattern generators
// ------------------------------------------------------------
void pattern0_wave() {
    static uint16_t counter = 0;
    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            uint16_t v = ((r + c + counter) % 32);
            colorGrid[r][c] = applyBrightness(v);
        }
    }
    counter++;
}

void pattern1_diagonal() {
    static uint16_t counter = 0;
    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            uint16_t v = ((r + counter) % 8 == c) ? 32 : 0;
            colorGrid[r][c] = applyBrightness(v);
        }
    }
    counter++;
}

void pattern2_checker() {
    static uint16_t counter = 0;
    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            uint16_t base = ((r + c + (counter / 8)) % 2) ? 32 : 0;
            colorGrid[r][c] = applyBrightness(base);
        }
    }
    counter++;
}

void updateGrid() {
    switch (patternIndex) {
        case 0: pattern0_wave();     break;
        case 1: pattern1_diagonal(); break;
        case 2: pattern2_checker();  break;
        default: pattern0_wave();    break;
    }
}

// ------------------------------------------------------------
// Command parsing
// ------------------------------------------------------------
void handleCommand(const String& msg) {
    Serial.println("CMD: " + msg);

    if (msg.startsWith("RUN:")) {
        animationRunning = msg.substring(4).toInt() != 0;
    }
    else if (msg.startsWith("FPS:")) {
        int fps = msg.substring(4).toInt();
        fps = constrain(fps, 1, 60);
        frameInterval = 1000UL / fps;
    }
    else if (msg.startsWith("PAT:")) {
        patternIndex = constrain(msg.substring(4).toInt(), 0, 2);
    }
    else if (msg.startsWith("BRI:")) {
        brightness = constrain(msg.substring(4).toFloat(), 0.0f, 1.0f);
    }
    else if (msg.startsWith("MODE:")) {
        String m = msg.substring(5);
        m.toUpperCase();
        sendMode = (m == "BIT") ? MODE_BITMASK : MODE_FULL;
    }
}

// ------------------------------------------------------------
// Multi-client viewer list + cleanup
// ------------------------------------------------------------
void printClientList() {
    String hostIP = WiFi.localIP().toString();
    Serial.printf("---- WebSocket Clients (Host: %s) ----\n", hostIP.c_str());

    for (AsyncWebSocketClient& c : ws.getClients()) {
        AsyncWebSocketClient* client = &c;

        IPAddress ip = client->remoteIP();
        uint32_t cid = client->id();
        String ua = clientUserAgents.count(cid) ? clientUserAgents[cid] : "Unknown";

        if (client->status() == WS_CONNECTED) {
            Serial.printf("Client %u: CONNECTED | IP: %s | UA: %s\n",
                          cid,
                          ip.toString().c_str(),
                          ua.c_str());
        } else {
            Serial.printf("Client %u: NOT CONNECTED (closing) | IP: %s | UA: %s\n",
                          cid,
                          ip.toString().c_str(),
                          ua.c_str());
            client->close();
        }
    }

    Serial.printf("Active client count (ws.count): %u\n", ws.count());
    Serial.println("-------------------------------------------");
}

// ------------------------------------------------------------
// Setup Web Server
// ------------------------------------------------------------
void setupWebServer() {
    if (!serveHTML("/index_grid.html")) {
        Serial.println("FATAL: Could not load /index_grid.html");
        return;
    }

    // Capture User-Agent during HTTP GET
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasHeader("User-Agent")) {
            String ua = request->getHeader("User-Agent")->value();
            uint32_t tempKey = request->client()->remotePort();
            clientUserAgents[tempKey] = ua;
        }

        AsyncWebServerResponse *response =
            request->beginResponse(200, "text/html",
                                   (const uint8_t*)htmlBuffer, htmlSize);
        response->addHeader("Cache-Control", "no-cache");
        request->send(response);
    });

    ws.onEvent([](AsyncWebSocket *server,
                  AsyncWebSocketClient *client,
                  AwsEventType type,
                  void *arg,
                  uint8_t *data,
                  size_t len) {

        if (type == WS_EVT_CONNECT) {
            uint32_t cid = client->id();
            IPAddress ip = client->remoteIP();

            // Move UA from temp key to real WebSocket ID
            if (clientUserAgents.count(client->remotePort())) {
                clientUserAgents[cid] = clientUserAgents[client->remotePort()];
                clientUserAgents.erase(client->remotePort());
            }

            Serial.printf("WS: Client %u connected | IP: %s | UA: %s\n",
                          cid,
                          ip.toString().c_str(),
                          clientUserAgents[cid].c_str());
        }

        if (type == WS_EVT_DISCONNECT) {
            Serial.printf("WS: Client %u disconnected\n", client->id());
            ws.cleanupClients();
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

                handleCommand(msg);
            }
        }
    });

    server.addHandler(&ws);
    server.begin();
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);

    LittleFS.begin();

    setupWiFi();
    setupWebServer();
}

// ------------------------------------------------------------
// OFFICIAL LOOP — always send frames when connected
// ------------------------------------------------------------
void loop() {
    unsigned long now = millis();

    if (animationRunning && (now - lastSend >= frameInterval)) {
        lastSend = now;

        updateGrid();

        if (ws.count() > 0) {
            if (sendMode == MODE_FULL) {
                sendFullGrid();
            } else {
                sendBitGrid();
            }
        }
    }

    // Periodic multi-client viewer list + deep cleanup
    static unsigned long lastClientPrint = 0;
    if (now - lastClientPrint > 5000) {
        lastClientPrint = now;
        printClientList();
        ws.cleanupClients();
    }
}
