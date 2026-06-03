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
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.println("\n[WiFi] Connected!");
            Serial.print("[WiFi] IP Address: ");
            Serial.println(WiFi.localIP());
            break;

        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("[WiFi] Lost connection, reconnecting...");
            WiFi.reconnect();
            break;

        default:
            break;
    }
}

// ------------------------------------------------------------
// WiFi Setup
// ------------------------------------------------------------
void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(WiFiEvent);
    WiFi.begin(ssid, password);
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
    if (!file) return false;

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

    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            if (colorGrid[r][c] != 0)
                bits |= (uint64_t)1 << (r * 8 + c);

    ws.binaryAll((uint8_t*)&bits, sizeof(bits));
}

// ------------------------------------------------------------
// Pattern generators
// ------------------------------------------------------------
void pattern0_wave() {
    static uint16_t counter = 0;
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            colorGrid[r][c] = ((r + c + counter) % 32) * brightness;
    counter++;
}

void pattern1_diagonal() {
    static uint16_t counter = 0;
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            colorGrid[r][c] = ((r + counter) % 8 == c ? 32 : 0) * brightness;
    counter++;
}

void pattern2_checker() {
    static uint16_t counter = 0;
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            colorGrid[r][c] = (((r + c + counter / 8) % 2) ? 32 : 0) * brightness;
    counter++;
}

void updateGrid() {
    switch (patternIndex) {
        case 0: pattern0_wave(); break;
        case 1: pattern1_diagonal(); break;
        case 2: pattern2_checker(); break;
    }
}

// ------------------------------------------------------------
// Command parsing
// ------------------------------------------------------------
void handleCommand(const String& msg, AsyncWebSocketClient* client) {

    if (msg.startsWith("UA:")) {
        clientUserAgents[client->id()] = msg.substring(3);
        Serial.printf("UA received for client %u\n", client->id());
        return;
    }

    if (msg.startsWith("RUN:")) animationRunning = msg.substring(4).toInt();
    else if (msg.startsWith("FPS:")) frameInterval = 1000UL / constrain(msg.substring(4).toInt(), 1, 60);
    else if (msg.startsWith("PAT:")) patternIndex = constrain(msg.substring(4).toInt(), 0, 2);
    else if (msg.startsWith("BRI:")) brightness = constrain(msg.substring(4).toFloat(), 0.0f, 1.0f);
    else if (msg.startsWith("MODE:")) sendMode = (msg.substring(5) == "BIT") ? MODE_BITMASK : MODE_FULL;
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
// Setup Web Server
// ------------------------------------------------------------
void setupWebServer() {
    if (!serveHTML("/index_grid.html")) {
        Serial.println("FATAL: Could not load HTML");
        return;
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
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
// Setup
// ------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    LittleFS.begin();
    setupWiFi();
    setupWebServer();
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------
void loop() {
    unsigned long now = millis();

    if (animationRunning && now - lastSend >= frameInterval) {
        lastSend = now;
        updateGrid();

        if (ws.count() > 0) {
            if (sendMode == MODE_FULL) sendFullGrid();
            else sendBitGrid();
        }
    }

    static unsigned long lastPrint = 0;
    if (now - lastPrint > 5000) {
        lastPrint = now;
        printClientList();
        ws.cleanupClients();
    }
}
