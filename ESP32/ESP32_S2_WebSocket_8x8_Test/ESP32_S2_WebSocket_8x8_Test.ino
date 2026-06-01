#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

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
uint8_t patternIndex = 0;           // 0,1,2...
float brightness = 1.0f;            // 0.0–1.0
enum SendMode { MODE_FULL, MODE_BITMASK };
SendMode sendMode = MODE_FULL;

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
// Send full 8×8 grid as 64 × uint16_t (128 bytes)
// ------------------------------------------------------------
void sendFullGrid() {
    ws.binaryAll((uint8_t*)colorGrid, sizeof(colorGrid));
}

// ------------------------------------------------------------
// Send compact 64-bit bitmask (uint64_t)
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
// Command parsing (simple text protocol)
// ------------------------------------------------------------
// Commands (text frames):
// RUN:0 or RUN:1
// FPS:<int>   (1–60)
// PAT:<int>   (0–2)
// BRI:<float> (0.0–1.0)
// MODE:FULL or MODE:BIT
// ------------------------------------------------------------
void handleCommand(const String& msg) {
    Serial.println("CMD: " + msg);

    if (msg.startsWith("RUN:")) {
        int v = msg.substring(4).toInt();
        animationRunning = (v != 0);
        Serial.printf("animationRunning = %d\n", animationRunning);
    }
    else if (msg.startsWith("FPS:")) {
        int fps = msg.substring(4).toInt();
        if (fps < 1) fps = 1;
        if (fps > 60) fps = 60;
        frameInterval = 1000UL / (unsigned long)fps;
        Serial.printf("FPS = %d, frameInterval = %lu ms\n", fps, frameInterval);
    }
    else if (msg.startsWith("PAT:")) {
        int p = msg.substring(4).toInt();
        if (p < 0) p = 0;
        if (p > 2) p = 2;
        patternIndex = (uint8_t)p;
        Serial.printf("patternIndex = %d\n", patternIndex);
    }
    else if (msg.startsWith("BRI:")) {
        float b = msg.substring(4).toFloat();
        if (b < 0.0f) b = 0.0f;
        if (b > 1.0f) b = 1.0f;
        brightness = b;
        Serial.printf("brightness = %.2f\n", brightness);
    }
    else if (msg.startsWith("MODE:")) {
        String m = msg.substring(5);
        m.toUpperCase();
        if (m == "FULL") {
            sendMode = MODE_FULL;
        } else if (m == "BIT") {
            sendMode = MODE_BITMASK;
        }
        Serial.printf("sendMode = %s\n", sendMode == MODE_FULL ? "FULL" : "BIT");
    }
}

// ------------------------------------------------------------
// Setup Web Server
// ------------------------------------------------------------
void setupWebServer() {
    if (!serveHTML("/index_grid.html")) {
        Serial.println("FATAL: Could not load /index_grid.html");
        return;
    }

    // Serve HTML from PSRAM
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response =
            request->beginResponse(200, "text/html",
                                   (const uint8_t*)htmlBuffer, htmlSize);
        response->addHeader("Cache-Control", "no-cache");
        request->send(response);
    });

    // WebSocket events
    ws.onEvent([](AsyncWebSocket *server,
                  AsyncWebSocketClient *client,
                  AwsEventType type,
                  void *arg,
                  uint8_t *data,
                  size_t len) {
        if (type == WS_EVT_CONNECT) {
            Serial.printf("WS: Client %u connected\n", client->id());
        }
        else if (type == WS_EVT_DISCONNECT) {
            Serial.printf("WS: Client %u disconnected\n", client->id());
        }
        else if (type == WS_EVT_DATA) {
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if (info->opcode == WS_TEXT) {
                String msg;
                for (size_t i = 0; i < len; i++) {
                    msg += (char)data[i];
                }
                handleCommand(msg);
            }
        }
    });

    server.addHandler(&ws);
    server.begin();

    Serial.println("Web server started");
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);

    if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed");
    } else {
        Serial.println("LittleFS mounted");
    }

    WiFi.begin(ssid, password);
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }
    Serial.println("\nConnected!");
    Serial.println(WiFi.localIP());

    setupWebServer();
}

// ------------------------------------------------------------
// Loop (Smooth, Continuous Animation with Command Control)
// ------------------------------------------------------------
void loop() {
    unsigned long now = millis();

    if (animationRunning && (now - lastSend >= frameInterval)) {
        lastSend = now;

        updateGrid();

        if (ws.count() > 0 && ws.availableForWriteAll()) {
            if (sendMode == MODE_FULL) {
                sendFullGrid();
            } else {
                sendBitGrid();
            }
        }
    }
}
