// Webserver game controller DPad web page code uploaded to Adafruit Forums by:
// Carlos Garcia (dancingevilgenius) on May 16, 2026
// Original board tested: Adafruit QT PY Pico
// Original Espressif library version:  3.3.8
// Original Adafruit Neopixel version: 1.15.5

#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>

// Network credentials Here
const char* ssid     = "STDL5301";	// Change this for your project
const char* password = "library30";	// Change this for your project
//const char* ssid     = "VERIZON-SM-G950U-DEA4";	// Change this for your project
//const char* password = "6302201111";	// Change this for your project

// Set web server port number to 80
NetworkServer server(80);

bool verbose = false; // Used to hide some of the less important web server connection properties.

// ----------------------------
// MENU SYSTEM (Hierarchical)
// ----------------------------

// Horizontal menu (fixed 4 items)
const char* horizontalMenu[] = { "SPEED", "TURNING", "PROPORTIONAL", "INTEGRAL" };
int horizontalIndex = 0;
const int horizontalCount = 4;

// Vertical lists for each horizontal menu
const char* verticalMenu_M1[] = { "50", "60", "70", "80", "90", "100" };
const char* verticalMenu_M2[] = { "50", "60", "70", "80", "90", "100" };
const char* verticalMenu_M3[] = { "50", "60", "70", "80", "90" };
const char* verticalMenu_M4[] = { "0.1", "0.2", "0.3", "0.4", "0.5" };

// Pointer array to vertical menus
const char** verticalMenus[] = {
  verticalMenu_M1,
  verticalMenu_M2,
  verticalMenu_M3,
  verticalMenu_M4
};

// Length of each vertical list
int verticalCounts[] = {
  sizeof(verticalMenu_M1) / sizeof(verticalMenu_M1[0]),
  sizeof(verticalMenu_M2) / sizeof(verticalMenu_M2[0]),
  sizeof(verticalMenu_M3) / sizeof(verticalMenu_M3[0]),
  sizeof(verticalMenu_M4) / sizeof(verticalMenu_M4[0])
};

int verticalIndex = 0;

// ----------------------------
// NAVIGATION FUNCTION
// ----------------------------
void navigateMenu(const String& direction) {

  if (direction == "left") {
    horizontalIndex--;
    if (horizontalIndex < 0)
      horizontalIndex = horizontalCount - 1;
    verticalIndex = 0;
  }

  else if (direction == "right") {
    horizontalIndex++;
    if (horizontalIndex >= horizontalCount)
      horizontalIndex = 0;
    verticalIndex = 0;
  }

  else if (direction == "up") {
    verticalIndex--;
    if (verticalIndex < 0)
      verticalIndex = verticalCounts[horizontalIndex] - 1;
  }

  else if (direction == "down") {
    verticalIndex++;
    if (verticalIndex >= verticalCounts[horizontalIndex])
      verticalIndex = 0;
  }

  else if (direction == "center") {
    Serial.println("Center pressed — select/confirm");
  }

  Serial.print("Menu: ");
  Serial.print(horizontalMenu[horizontalIndex]);
  Serial.print(" | Item: ");
  Serial.println(verticalMenus[horizontalIndex][verticalIndex]);
}


void setup() {
  Serial.begin(115200);

  delay(2000); // Fixes problem that displays ONLY firmware debugging info.

  WiFi.begin(ssid,password);
  int count=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Attempting to connect. count:");
    Serial.println(count++);
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();

}


void loop() {
  NetworkClient client = server.available();
  if (!client) return;

  String request = "";
  unsigned long timeout = millis();

  while (client.connected() && millis() - timeout < 2000) {
    if (client.available()) {
      char c = client.read();
      request += c;
      if (request.endsWith("\r\n\r\n")) break;
    }
  }

  if (request.startsWith("GET / ") || request.startsWith("GET /index.html")) {
    sendWebPage(client);
    client.stop();
    return;
  }

  if (request.startsWith("POST /controller")) {

    int clIndex = request.indexOf("Content-Length:");
    int contentLength = 0;
    if (clIndex != -1) {
      int start = clIndex + 15;
      int end = request.indexOf("\r\n", start);
      contentLength = request.substring(start, end).toInt();
    }

    String body = "";
    while (client.available() < contentLength) delay(1);
    while (client.available()) body += (char)client.read();

    Serial.println("=== JSON BODY RECEIVED ===");
    Serial.println(body);

    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error) {
      Serial.print("JSON parse failed: ");
      Serial.println(error.c_str());
    } else {

      // ----------------------------
      // HANDLE DIRECTION
      // ----------------------------
      if (doc.containsKey("direction")) {
        const char* direction = doc["direction"];
        Serial.print("Direction: ");
        Serial.println(direction);
        navigateMenu(direction);
      }

      // ----------------------------
      // HANDLE START / STOP ACTIONS
      // ----------------------------
      if (doc.containsKey("action")) {
        const char* action = doc["action"];
        Serial.print("Action: ");
        Serial.println(action);

        if (strcmp(action, "start") == 0) {
          Serial.println(">>> START triggered");
        }
        else if (strcmp(action, "stop") == 0) {
          Serial.println(">>> STOP triggered");
        }
      }

      // ----------------------------
      // HANDLE DROPDOWN MENU
      // ----------------------------
      if (doc.containsKey("menu")) {
        const char* menuValue = doc["menu"];
        Serial.print("Dropdown selected: ");
        Serial.println(menuValue);

        // You can add logic here to react to menu selection
      }
    }

    // Build JSON response
    StaticJsonDocument<200> response;
    response["horiz"] = horizontalMenu[horizontalIndex];
    response["vert"]  = verticalMenus[horizontalIndex][verticalIndex];

    String out;
    serializeJson(response, out);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.println(out);
    client.stop();
    return;
  }

  client.println("HTTP/1.1 400 Bad Request");
  client.println("Connection: close");
  client.println();
  client.stop();  
}

void sendResponseHeader(NetworkClient client) {
    
    // Should not normally edit/remove these 4 lines
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();

}

void sendWebPage(NetworkClient client){

  // Send your web page here.
  // In this case it is a simulated game controller DPad.
  clientDPad(client);
}

void clientDPad(NetworkClient client){

client.println("<!DOCTYPE html>");
client.println("<html>");
client.println("<head>");
client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
client.println("<title>Web D-Pad</title>");

client.println("<style>");
client.println("body {");
client.println("    font-family: Arial, sans-serif;");
client.println("    background: #111;");
client.println("    color: #eee;");
client.println("    margin: 0;");
client.println("    padding: 0;");
client.println("    text-align: center;");
client.println("}");
client.println(".center-column {");
client.println("    margin-top: 20px;");
client.println("}");
client.println("select {");
client.println("    font-size: 20px;");
client.println("    padding: 8px;");
client.println("    border-radius: 8px;");
client.println("    margin-bottom: 10px;");
client.println("}");
client.println(".status-row {");
client.println("    display: flex;");
client.println("    justify-content: center;");
client.println("    gap: 20px;");
client.println("    margin-bottom: 20px;");
client.println("}");
client.println(".status-box {");
client.println("    width: 140px;");
client.println("    height: 40px;");
client.println("    background: #222;");
client.println("    border: 1px solid #555;");
client.println("    border-radius: 6px;");
client.println("    color: #fff;");
client.println("    font-size: 18px;");
client.println("    text-align: center;");
client.println("    line-height: 40px;");
client.println("}");
client.println(".dpad-container {");
client.println("    margin-top: 10px;");
client.println("}");
client.println(".row {");
client.println("    display: flex;");
client.println("    justify-content: center;");
client.println("}");
client.println(".btn {");
client.println("    width: 80px;");
client.println("    height: 80px;");
client.println("    margin: 6px;");
client.println("    border-radius: 12px;");
client.println("    border: none;");
client.println("    background: #333;");
client.println("    color: #fff;");
client.println("    font-size: 28px;");
client.println("    cursor: pointer;");
client.println("}");
client.println(".btn:active { background: #555; }");
client.println(".bottom-buttons {");
client.println("    margin-top: 40px;");
client.println("    margin-bottom: 20px;");
client.println("    display: flex;");
client.println("    justify-content: center;");
client.println("    gap: 20px;");
client.println("}");
client.println(".action-btn {");
client.println("    width: 140px;");
client.println("    height: 60px;");
client.println("    border-radius: 10px;");
client.println("    border: none;");
client.println("    background: #444;");
client.println("    color: #fff;");
client.println("    font-size: 24px;");
client.println("    cursor: pointer;");
client.println("}");
client.println(".action-btn:active { background: #666; }");
client.println("</style>");

client.println("<script>");
client.println("function sendDirection(dir) {");
client.println("    fetch('/controller', {");
client.println("        method: 'POST',");
client.println("        headers: { 'Content-Type': 'application/json' },");
client.println("        body: JSON.stringify({ direction: dir })");
client.println("    })");
client.println("    .then(r => r.json())");
client.println("    .then(data => {");
client.println("        document.getElementById('hStatus').innerText = data.horiz;");
client.println("        document.getElementById('vStatus').innerText = data.vert;");
client.println("    });");
client.println("}");
client.println("function sendAction(action) {");
client.println("    fetch('/controller', {");
client.println("        method: 'POST',");
client.println("        headers: { 'Content-Type': 'application/json' },");
client.println("        body: JSON.stringify({ action: action })");
client.println("    });");
client.println("}");
client.println("</script>");

client.println("</head>");
client.println("<body>");

client.println("<div class='center-column'>");
client.println("    <select id='dropdown'>");
client.println("        <option value='item1' selected>Menu Item 1</option>");
client.println("        <option value='item2'>Menu Item 2</option>");
client.println("        <option value='item3'>Menu Item 3</option>");
client.println("    </select>");

client.println("    <div class='status-row'>");
client.println("        <div id='hStatus' class='status-box'></div>");
client.println("        <div id='vStatus' class='status-box'></div>");
client.println("    </div>");
client.println("</div>");

client.println("<div class='dpad-container'>");
client.println("    <div class='row'>");
client.println("        <button class='btn' onclick=\"sendDirection('up')\">&#9650;</button>");
client.println("    </div>");

client.println("    <div class='row'>");
client.println("        <button class='btn' onclick=\"sendDirection('left')\">&#9664;</button>");
client.println("        <button class='btn' onclick=\"sendDirection('center')\">OK</button>");
client.println("        <button class='btn' onclick=\"sendDirection('right')\">&#9654;</button>");
client.println("    </div>");

client.println("    <div class='row'>");
client.println("        <button class='btn' onclick=\"sendDirection('down')\">&#9660;</button>");
client.println("    </div>");
client.println("</div>");

client.println("<div class='bottom-buttons'>");
client.println("    <button class='action-btn' onclick=\"sendAction('start')\">Start</button>");
client.println("    <button class='action-btn' onclick=\"sendAction('stop')\">Stop</button>");
client.println("</div>");

client.println("</body>");
client.println("</html>");

}

String getParam(String request, String key) {
    int keyIndex = request.indexOf(key + "=");
    if (keyIndex == -1) return "";

    int start = keyIndex + key.length() + 1;
    int end = request.indexOf('&', start);
    if (end == -1) end = request.indexOf(' ', start);

    return request.substring(start, end);
}

void handleClientRequest(String request) {

  handleRequestParamDirection(request); // DPad sends form data as 'direction' param.
}

void handleRequestParamDirection(String request){
  String direction = getParam(request, "direction");

  String dirSet[] = {"up", "down", "left", "right", "center"};

  int setSize = 5;
  bool found = false;

  for (int i = 0 ; i < setSize ; i++) {
    if (direction == dirSet[i]) {
      found = true;
      break; // Exit loop early once match is found
    }
  }

  //if (direction.length() > 0) {
  if(found){
    Serial.print("Direction pressed: ");
    Serial.println(direction);

    if (direction == "up") {
      
    }
    else if (direction == "down") {
      
    }
    else if (direction == "left") {
      
    }
    else if (direction == "right") {
      
    }
    else if (direction == "center") {
      
    }
    

  }

}

