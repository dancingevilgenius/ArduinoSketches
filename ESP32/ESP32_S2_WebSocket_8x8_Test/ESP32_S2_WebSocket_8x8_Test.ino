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
// Example pattern generator
// ------------------------------------------------------------
void updateGrid() {
    static uint16_t counter = 0;

    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            colorGrid[r][c] = (r + c + counter) % 32;
        }
    }

    counter++;
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
    });

    server.addHandler(&ws);
    server.begin();

    Serial.println("Web server started");
}

// ------------------------------------------------------------
// Smooth Animation Timer
// ------------------------------------------------------------
unsigned long lastSend = 0;
const unsigned long frameInterval = 100;  // 10 Hz

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);

    // Init LittleFS
    if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed");
    } else {
        Serial.println("LittleFS mounted");
    }

    // WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }
    Serial.println("\nConnected!");
    Serial.println(WiFi.localIP());

    // Start server
    setupWebServer();
}

// ------------------------------------------------------------
// Loop (Smooth, Continuous Animation)
// ------------------------------------------------------------
void loop() {
    unsigned long now = millis();

    if (now - lastSend >= frameInterval) {
        lastSend = now;

        updateGrid();

        // Only send if WebSocket is ready (prevents freezing)
        if (ws.count() > 0 && ws.availableForWriteAll()) {
            sendFullGrid();
            // sendBitGrid();  // optional
        }
    }
}
