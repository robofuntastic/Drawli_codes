#include <WiFi.h>
#include <WebServer.h>

const char *ssid = "RoboFuntastic";
const char *password = "123456789";

WebServer server(80);

void handleRoot() {
  server.send(200, "text/plain", "hello to you");
}

void handleButtonPress() {
  server.send(200, "text/plain", "hi from web");
}

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.softAP(ssid, password, 1, 0);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Route setup
  server.on("/", HTTP_GET, handleRoot);
  server.on("/button", HTTP_POST, handleButtonPress);

  // Start server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  WiFiClient client = server.client();
  if (client.connected()) {
    Serial.println("Client connected");
    while (client.connected()) {
      if (client.available()) {
        String line = client.readStringUntil('\r');
        Serial.print("Received from client: ");
        Serial.println(line);
        client.println("Received from client: ");
        client.println(line);
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}
