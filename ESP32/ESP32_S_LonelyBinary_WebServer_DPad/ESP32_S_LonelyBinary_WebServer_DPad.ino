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
//const char* ssid     = "STDL5301";	// Change this for your project
//const char* password = "library30";	// Change this for your project
//const char* ssid     = "TheMandalorian";	// Change this for your project
//const char* password = "6302201111";	// Change this for your project
const char* ssid     = "TheMandaloriKen";	// Change this for your project
const char* password = "asdf12346302201111";	// Change this for your project

// Set web server port number to 80
NetworkServer server(80);

bool verbose = false; // Used to hide some of the less important web server connection properties.


#define BUILTIN_LED 15 // ESP32-S2 Mini

// -- For Sensors loop
#define SENSOR_INTERVAL_TIME 1000   // milliseconds
long lastSensorUpdateTime = 0;


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


// ---------------------
//  8x8 web grid colors
// ---------------------
#define RED_OFFSET    0
#define GREEN_OFFSET  10
unsigned int gridColors[8][8];
int currentRow = 3;
int currentCol = 5;


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
  // move forward in list
  verticalIndex++;
  if (verticalIndex >= verticalCounts[horizontalIndex])
    verticalIndex = 0;  // wrap to first
}

else if (direction == "down") {
  // move backward in list
  verticalIndex--;
  if (verticalIndex < 0)
    verticalIndex = verticalCounts[horizontalIndex] - 1; // wrap to last
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
  blink(1, 2000, 1000);

  WiFi.begin(ssid,password);
  int count=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Attempting to connect. count:");
    Serial.println(count++);
    blink(6, 200, 200);
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  server.begin();

  setupGridColors();
}

void setupGridColors() {

  // Bottom red gradient (unchanged)
  gridColors[7][0] = RED_OFFSET + 0;
  gridColors[7][1] = RED_OFFSET + 1;
  gridColors[7][2] = RED_OFFSET + 2;
  gridColors[7][3] = RED_OFFSET + 3;
  gridColors[7][4] = RED_OFFSET + 4;
  gridColors[7][5] = RED_OFFSET + 5;
  gridColors[7][6] = RED_OFFSET + 6;
  gridColors[7][7] = RED_OFFSET + 7;

  // Only ONE green cell now
  gridColors[3][5] = GREEN_OFFSET + 7;

  // Track its starting position
  currentRow = 3;
  currentCol = 5;

  Serial.println("setupGridColors() completed. Initial display pattern");
}

void loopWebController() {

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

    //Serial.println("=== JSON BODY RECEIVED ===");
    //Serial.println(body);

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
      }

      // ----------------------------
      // FRONTEND PERIODIC GRID REQUEST
      // ----------------------------
      if (doc.containsKey("requestGrid")) {

          StaticJsonDocument<700> response;

          JsonArray grid = response.createNestedArray("grid");
          for (int r = 0; r < 8; r++) {
              JsonArray row = grid.createNestedArray();
              for (int c = 0; c < 8; c++) {
                  row.add(gridColors[r][c]);
              }
          }

          String out;
          serializeJson(response, out);

          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: application/json");
          client.println("Connection: close");
          client.println();
          client.println(out);
          return;
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


// List of the 6 original coordinates
int coords[6][2] = { 
      {2,3}, {2,4}, {2,5},
      {3,3}, {3,4}, {3,5}
};

// Temporary array to store new positions
int newCoords[6][2];


void loopSensors() {

  loop8x8Sensors();

}

void loop8x8Sensors() {
  long now = millis();
  long dt = now - lastSensorUpdateTime;

  if (dt < SENSOR_INTERVAL_TIME) return;


  // -----------------------------------------
  // MOVE ONLY THE TRACKED GREEN CELL LEFT
  // -----------------------------------------

  // Get the current value
  unsigned int value = gridColors[currentRow][currentCol];

  // Clear old position
  gridColors[currentRow][currentCol] = 0;

  // Compute new column with wrap
  int newCol = currentCol - 1;
  if (newCol < 0) newCol = 7;

  // Write new position
  gridColors[currentRow][newCol] = value;

  // Update global tracker
  currentCol = newCol;

  //Serial.print("Handle sensors. newCol:");
  //Serial.println(newCol);

  // Update timestamp
  lastSensorUpdateTime = now;
}

void loop() {

  loopSensors();
  loopWebController();
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
client.println("body { font-family: Arial, sans-serif; background: #111; color: #eee; margin: 0; padding: 0; text-align: center; }");
client.println(".center-column { margin-top: 20px; }");
client.println("select { font-size: 20px; padding: 8px; border-radius: 8px; margin-bottom: 10px; }");
client.println(".status-row { display: flex; justify-content: center; gap: 20px; margin-bottom: 20px; }");
client.println(".status-box { width: 140px; height: 40px; background: #222; border: 1px solid #555; border-radius: 6px; color: #fff; font-size: 18px; text-align: center; line-height: 40px; }");

client.println(".dpad-container { margin-top: 10px; }");
client.println(".row { display: flex; justify-content: center; }");
client.println(".btn { width: 80px; height: 80px; margin: 6px; border-radius: 12px; border: none; background: #333; color: #fff; font-size: 28px; cursor: pointer; }");
client.println(".btn:active { background: #555; }");

client.println(".bottom-buttons { margin-top: 40px; margin-bottom: 20px; display: flex; justify-content: center; gap: 20px; }");
client.println(".action-btn { width: 140px; height: 60px; border-radius: 10px; border: none; background: #444; color: #fff; font-size: 24px; cursor: pointer; }");
client.println(".action-btn:active { background: #666; }");

client.println(".grid-container { margin-top: 20px; }");
client.println(".grid { display: grid; grid-template-columns: repeat(8, 40px); grid-template-rows: repeat(8, 40px); gap: 4px; }");
client.println(".grid-cell { width: 40px; height: 40px; border-radius: 4px; background: #000; }");
client.println("</style>");

client.println("<script>");

// ------------------------------
// SEND DIRECTION
// ------------------------------
client.println("function sendDirection(dir) {");
client.println("  fetch('/controller', {");
client.println("    method: 'POST',");
client.println("    headers: { 'Content-Type': 'application/json' },");
client.println("    body: JSON.stringify({ direction: dir })");
client.println("  })");
client.println("  .then(r => r.json())");
client.println("  .then(data => {");
client.println("    document.getElementById('hStatus').innerText = data.horiz;");
client.println("    document.getElementById('vStatus').innerText = data.vert;");
client.println("    if (data.grid) updateGrid(data.grid);");
client.println("  });");
client.println("}");

// ------------------------------
// SEND ACTION
// ------------------------------
client.println("function sendAction(action) {");
client.println("  fetch('/controller', {");
client.println("    method: 'POST',");
client.println("    headers: { 'Content-Type': 'application/json' },");
client.println("    body: JSON.stringify({ action: action })");
client.println("  })");
client.println("  .then(r => r.json())");
client.println("  .then(data => { if (data.grid) updateGrid(data.grid); });");
client.println("}");

// ------------------------------
// UPDATE GRID COLORS
// ------------------------------
client.println("function updateGrid(cells) {");
client.println("  const grid = document.querySelectorAll('.grid-cell');");
client.println("  let index = 0;");
client.println("  for (let r = 0; r < 8; r++) {");
client.println("    for (let c = 0; c < 8; c++) {");
client.println("      const val = cells[r][c];");
client.println("      let color = '#000';");

client.println("      if (val === 0) {");
client.println("        color = 'rgb(0,0,0)';");
client.println("      } else if (val >= 1 && val <= 7) {");
client.println("        const shade = Math.min(255, 30 + val * 30);");
client.println("        color = `rgb(${shade},0,0)`;");
client.println("      } else if (val >= 10 && val <= 17) {");
client.println("        const level = val - 10;");
client.println("        const shade = Math.min(255, 30 + level * 30);");
client.println("        color = `rgb(0,${shade},0)`;");
client.println("      } else {");
client.println("        color = '#222';");
client.println("      }");

client.println("      grid[index].style.background = color;");
client.println("      index++;");
client.println("    }");
client.println("  }");
client.println("}");

client.println("setInterval(() => {");
client.println("  const mode = document.getElementById('dropdown').value;");
client.println("  if (mode !== '8x8') return;");   // Only poll in 8x8 mode");
client.println("");
client.println("  fetch('/controller', {");
client.println("    method: 'POST',");
client.println("    headers: { 'Content-Type': 'application/json' },");
client.println("    body: JSON.stringify({ requestGrid: true })");
client.println("  })");
client.println("  .then(r => r.json())");
client.println("  .then(data => { if (data.grid) updateGrid(data.grid); });");
client.println("}, 500);");


// ------------------------------
// DROPDOWN TOGGLE
// ------------------------------
client.println("document.addEventListener('DOMContentLoaded', () => {");
client.println("  const dropdown = document.getElementById('dropdown');");
client.println("  const dpad = document.querySelector('.dpad-container');");
client.println("  const grid = document.getElementById('grid8x8');");

client.println("  dropdown.addEventListener('change', function() {");
client.println("    if (this.value === '8x8') { dpad.style.display = 'none'; grid.style.display = 'block'; }");
client.println("    else { dpad.style.display = 'block'; grid.style.display = 'none'; }");
client.println("  });");
client.println("});");

client.println("</script>");
client.println("</head>");
client.println("<body>");

client.println("<div class='center-column'>");
client.println("  <select id='dropdown'>");
client.println("    <option value='dpad' selected>DPad</option>");
client.println("    <option value='8x8'>8x8</option>");
client.println("    <option value='tba'>TBA</option>");
client.println("  </select>");

client.println("  <div class='status-row'>");
client.println("    <div id='hStatus' class='status-box'></div>");
client.println("    <div id='vStatus' class='status-box'></div>");
client.println("  </div>");
client.println("</div>");

client.println("<div class='dpad-container'>");
client.println("  <div class='row'><button class='btn' onclick=\"sendDirection('up')\">&#9650;</button></div>");
client.println("  <div class='row'>");
client.println("    <button class='btn' onclick=\"sendDirection('left')\">&#9664;</button>");
client.println("    <button class='btn' onclick=\"sendDirection('center')\">OK</button>");
client.println("    <button class='btn' onclick=\"sendDirection('right')\">&#9654;</button>");
client.println("  </div>");
client.println("  <div class='row'><button class='btn' onclick=\"sendDirection('down')\">&#9660;</button></div>");
client.println("</div>");

client.println("<div id='grid8x8' class='grid-container' style='display:none;'>");
client.println("  <div class='row'>");
client.println("    <div class='grid'>");

for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
        client.println("      <div class='grid-cell'></div>");
    }
}

client.println("    </div>");
client.println("  </div>");
client.println("</div>");

client.println("<div class='bottom-buttons'>");
client.println("  <button class='action-btn' onclick=\"sendAction('start')\">Start</button>");
client.println("  <button class='action-btn' onclick=\"sendAction('stop')\">Stop</button>");
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

void blink(int count, int interval, int wait_ms){
    for(int i=0 ; i<count ; i++){
        digitalWrite(BUILTIN_LED, HIGH);
        delay(interval);
        digitalWrite(BUILTIN_LED, LOW);
        delay(interval);
    }

    delay(wait_ms);
}


