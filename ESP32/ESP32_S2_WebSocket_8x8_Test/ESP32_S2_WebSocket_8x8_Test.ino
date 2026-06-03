#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <map>

const char* ssid = "TheMandaloriKen";
const char* password = "asdf12346302201111";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

std::map<uint32_t, String> clientUserAgents;

uint16_t colorGrid[8][8];

char* htmlBuffer = nullptr;
size_t htmlSize = 0;

bool animationRunning = true;
unsigned long lastSend = 0;
unsigned long frameInterval = 100;
uint8_t patternIndex = 0;
float brightness = 1.0f;
enum SendMode { MODE_FULL, MODE_BITMASK };
SendMode sendMode = MODE_FULL;

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
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

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(WiFiEvent);
    WiFi.begin(ssid, password);
}

bool serveHTML(const char* filename) {
    if (!LittleFS.exists(filename)) return false;

    File file = LittleFS.open(filename, "r");
    htmlSize = file.size();
    htmlBuffer = (char*)ps_malloc(htmlSize + 1);
    file.readBytes(htmlBuffer, htmlSize);
    htmlBuffer[htmlSize] = '\0';
    file.close();
    return true;
}

void sendFullGrid() {
    ws.binaryAll((uint8_t*)colorGrid, sizeof(colorGrid));
}

void sendBitGrid() {
    uint64_t bits = 0;
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            if (colorGrid[r][c] != 0)
                bits |= (uint64_t)1 << (r * 8 + c);

    ws.binaryAll((uint8_t*)&bits, sizeof(bits));
}

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

void handleCommand(const String& msg, AsyncWebSocketClient* client) {
    if (msg.startsWith("UA:")) {
        clientUserAgents[client->id()] = msg.substring(3);
        Serial.printf("UA received for client %u: %s\n",
                      client->id(),
                      clientUserAgents[client->id()].c_str());
        return;
    }

    if (msg.startsWith("RUN:")) animationRunning = msg.substring(4).toInt();
    else if (msg.startsWith("FPS:")) frameInterval = 1000UL / constrain(msg.substring(4).toInt(), 1, 60);
    else if (msg.startsWith("PAT:")) patternIndex = constrain(msg.substring(4).toInt(), 0, 2);
    else if (msg.startsWith("BRI:")) brightness = constrain(msg.substring(4).toFloat(), 0.0f, 1.0f);
    else if (msg.startsWith("MODE:")) sendMode = (msg.substring(5) == "BIT") ? MODE_BITMASK : MODE_FULL;
}

void printClientList() {
    String hostIP = WiFi.localIP().toString();
    Serial.printf("---- WebSocket Clients (Host: %s) ----\n", hostIP.c_str());

    for (AsyncWebSocketClient& c : ws.getClients()) {
        AsyncWebSocketClient* client = &c;
        uint32_t cid = client->id();
        IPAddress ip = client->remoteIP();
        String ua = clientUserAgents.count(cid) ? clientUserAgents[cid] : "Unknown";

        if (client->status() == WS_CONNECTED) {
            Serial.printf("Client %u: CONNECTED | IP: %s | UA: %s\n",
                          cid, ip.toString().c_str(), ua.c_str());
        } else {
            Serial.printf("Client %u: NOT CONNECTED (closing) | IP: %s | UA: %s\n",
                          cid, ip.toString().c_str(), ua.c_str());
            client->close();
        }
    }

    Serial.printf("Active client count: %u\n", ws.count());
    Serial.println("-------------------------------------------");
}

void setupWebServer() {
    serveHTML("/index_grid.html");

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

void setup() {
    Serial.begin(115200);
    LittleFS.begin();
    setupWiFi();
    setupWebServer();
}

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
