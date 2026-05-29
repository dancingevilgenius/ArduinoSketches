#include <WiFi.h>
#include <LittleFS.h>


//const char* ssid     = "STDL5301";    const char* password = "library30";	// Change this for your project
//const char* ssid     = "TheMandalorian";  const char* password = "6302201111";	// Change this for your project
const char* ssid     = "TheMandaloriKen";   const char* password = "asdf12346302201111";	// Change this for your project


WiFiServer server(80);

// PSRAM buffer for index.html
char* htmlPage = nullptr;
size_t htmlSize = 0;

bool loadIndexHtmlToPSRAM() {
    File file = LittleFS.open("/index.html", "r");
    if (!file) {
        Serial.println("Failed to open /index.html from LittleFS");
        return false;
    }

    htmlSize = file.size();
    if (htmlSize == 0) {
        Serial.println("/index.html is empty");
        file.close();
        return false;
    }

    // Allocate PSRAM
    htmlPage = (char*)ps_malloc(htmlSize + 1);
    if (!htmlPage) {
        Serial.println("ps_malloc failed (no PSRAM?)");
        file.close();
        return false;
    }

    // Read file into PSRAM
    size_t readBytes = file.readBytes(htmlPage, htmlSize);
    file.close();

    if (readBytes != htmlSize) {
        Serial.println("Failed to read full index.html into PSRAM");
        free(htmlPage);
        htmlPage = nullptr;
        htmlSize = 0;
        return false;
    }

    htmlPage[htmlSize] = '\0'; // null-terminate for safety
    Serial.printf("Loaded /index.html into PSRAM (%u bytes)\n", (unsigned)htmlSize);
    return true;
}

void serveHTML(WiFiClient &client) {
    if (!htmlPage || htmlSize == 0) {
        client.println("HTTP/1.1 500 Internal Server Error");
        client.println("Content-Type: text/plain");
        client.println("Connection: close");
        client.println();
        client.println("index.html not loaded");
        return;
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.write(htmlPage, htmlSize);
}

void sendOK(WiFiClient &client, const char* msg) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println(msg);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Check PSRAM
    if (!psramFound()) {
        Serial.println("PSRAM not found! Make sure it's enabled in board settings.");
    } else {
        Serial.println("PSRAM detected.");
    }
    delay(1000);

    // Mount LittleFS
    if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed. Did you upload index.html?");
    } else {
        Serial.println("LittleFS mounted.");
    }
    delay(1000);

    // Load index.html from LittleFS into PSRAM
    if (!loadIndexHtmlToPSRAM()) {
        Serial.println("Failed to load index.html into PSRAM.");
    }
    delay(1000);

    // WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    delay(1000);
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    server.begin();
}

void loop() {
    WiFiClient client = server.available();
    if (!client) return;

    // Simple request line parsing
    String req = client.readStringUntil('\r');
    client.readStringUntil('\n'); // consume newline

    if (req.startsWith("GET / ")) {
        serveHTML(client);
    }
    else if (req.startsWith("GET /start")) {
        Serial.println("Start pressed");
        sendOK(client, "Started");
    }
    else if (req.startsWith("GET /stop")) {
        Serial.println("Stop pressed");
        sendOK(client, "Stopped");
    }
    else {
        client.println("HTTP/1.1 404 Not Found");
        client.println("Connection: close");
        client.println();
    }

    client.stop();
}