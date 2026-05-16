#include <WiFi.h>
#include <WebServer.h>
#include "FS.h"
#include "LittleFS.h"

// --- CONFIGURATION ---
const char* ssid     = "STDL5301";     // Replace with your Wi-Fi Name
const char* password = "library30"; // Replace with your Wi-Fi Password

WebServer server(80); // Start web server on port 80 (standard HTTP port)

// Global pointer to track our HTML data location inside the PSRAM
char* globalHtmlBuffer = nullptr; 

// --- FUNCTIONS ---

// Load file from Flash filesystem into physical PSRAM
bool loadHtmlToPSRAM() {
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS Mount Failed");
    return false;
  }

  File file = LittleFS.open("/index.html", "r");
  if (!file || file.isDirectory()) {
    Serial.println("[FS] Failed to open index.html from Flash!");
    return false;
  }

  size_t fileSize = file.size();
  Serial.printf("[FS] Found index.html size: %d bytes\n", fileSize);

  // Allocate dynamic space on the external PSRAM chip (+1 byte for terminal character)
  globalHtmlBuffer = (char *)ps_malloc(fileSize + 1);
  
  if (globalHtmlBuffer == nullptr) {
    Serial.println("[PSRAM] Buffer allocation failed!");
    file.close();
    return false;
  }

  // Stream data from storage partition directly into the PSRAM memory block
  size_t bytesRead = file.readBytes(globalHtmlBuffer, fileSize);
  globalHtmlBuffer[bytesRead] = '\0'; // Append standard C-string null terminator

  file.close();
  Serial.println("[PSRAM] HTML file cached securely in PSRAM.");
  return true;
}

// Request handler that fires whenever a client visits the root URL "/"
void handleRootRequest() {
  if (globalHtmlBuffer != nullptr) {
    // Serve the webpage content directly out of physical PSRAM 
    server.send(200, "text/html", globalHtmlBuffer);
    Serial.println("[Server] Served index.html from PSRAM to a client!");
  } else {
    server.send(500, "text/plain", "Internal Server Error: Page not loaded in RAM.");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000); // Wait briefly for USB Serial interface

  Serial.println("\n--- QT PY Webpage from FS ---");
  Serial.println("\n--- Initiating Server Setup ---");

  // 1. Extract the asset layout from LittleFS into PSRAM
  if (!loadHtmlToPSRAM()) {
    Serial.println("System halt: Failed to resolve core assets.");
    while (true) delay(1000); 
  }

  // 2. Initialize Wi-Fi Station connection
  Serial.printf("Connecting to Network: %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n[Wi-Fi] Connection successful!");
  Serial.print("[Wi-Fi] Board Local IP Address: ");
  Serial.println(WiFi.localIP()); // Print the IP address to type into your browser

  // 3. Bind the routing paths to our handler logic
  server.on("/", handleRootRequest);

  // 4. Fire up the HTTP engine listener
  server.begin();
  Serial.println("[Server] HTTP listener started successfully!");
}

void loop() {
  // Constantly parse incoming network requests from active clients
  server.handleClient();
  delay(2); // Yield to the underlying ESP32 RTOS core manager
}
