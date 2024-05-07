#include <WiFi.h>
#include <HardwareSerial.h>

#define TX_PIN 16 // GPIO1
#define RX_PIN 17 // GPIO3

const char *ssid = "RoboFuntastic";
const char *password = "123456789";
WiFiServer server(80);
WiFiClient client;

void setup() {
  Serial.begin(115200,SERIAL_8N1, TX_PIN, RX_PIN);
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

            if (command.startsWith("str")) {
          delay(1000); // Wait for 1 second
          client.println("str_ok");
          Serial.println("Sent: str_ok");
        }

            if (command.startsWith("rot")) {
          delay(1000); // Wait for 1 second
          client.println("rot_ok");
          Serial.println("Sent: rot_ok");
        }

            if (command.startsWith("cir")) {
          delay(1000); // Wait for 1 second
          client.println("cir_ok");
          Serial.println("Sent: cir_ok");
        }
    
    }
  } }

}


//ESP32 
