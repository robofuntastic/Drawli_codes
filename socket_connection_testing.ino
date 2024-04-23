#include <WiFi.h>

const char *ssid = "RoboFuntastic";
const char *password = "123456789";
WiFiServer server(80);
WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Configuring access point...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.begin();
  
}

void loop() {
 WiFiClient client = server.available();   // listen for incoming clients


  if (client) {
    while (client.connected()) { 
    if (client.available()) {
      String command = client.readStringUntil('\r');
      Serial.println("Received command: " + command);
      // Process the command here (e.g., move motors based on the received command)
    }
  } 
  }

}


//ESP32 
