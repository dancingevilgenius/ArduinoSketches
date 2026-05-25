#include <LittleFS.h>
#include <WiFi.h> 


// Network credentials Here
//const char* ssid     = "STDL5301";	// Change this for your project
//const char* password = "library30";	// Change this for your project
const char* ssid     = "TheMandalorian";	// Change this for your project
const char* password = "6302201111";	// Change this for your project

NetworkServer server(80);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("LittleFS Test start.");
  // 1. Start LittleFS
  if(!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  WiFi.begin(ssid,password);
  int count=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Attempting to connect. count:" + count++);
  }



  // Connect to network...
  server.begin();

  Serial.println("LittleFS Test start.");

}

void loop() {
  NetworkClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        String req = client.readStringUntil('\r');
        client.flush();

        // 2. Simple route for "/" or "/index.html"
        if (req.indexOf("GET /") != -1) {
          File file = LittleFS.open("/index.html", "r");
          if (file) {
            // 3. Send HTTP Headers
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");
            client.println();

            // 4. Stream the file content
            while (file.available()) {
              client.write(file.read());
            }
            file.close();
          } else {
            client.println("HTTP/1.1 404 Not Found");
          }
        }
        break;
      }
    }
    client.stop();
  }
}
