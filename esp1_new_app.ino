#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "driver/uart.h"
#include <FastLED.h>

#define TX_PIN 16
#define RX_PIN 17
#define UART_PORT UART_NUM_1

#define LED_PIN     25
#define LED_2_PIN   26
#define NUM_LEDS    3
CRGB leds[NUM_LEDS];
CRGB leds2[NUM_LEDS];

const char* ssid = "RoboFuntastic";
const char* password = "123456789";

String command;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setupUART() {
  // Configure UART parameters
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  // Install UART driver and configure the UART
  uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
  uart_param_config(UART_PORT, &uart_config);
  uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


void sendUARTData(const char* data) {
  // Create a new string with a newline added to the end of the data
  String dataWithNewline = String(data) + "\n";
  
  // Send the data packet via UART
  uart_write_bytes(UART_PORT, dataWithNewline.c_str(), dataWithNewline.length());
  
  // Optional: Debugging output to check what was sent
  // Serial.printf("Sent via UART: %s\n", dataWithNewline.c_str());
}


void setLEDs(int red, int green, int blue) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(red, green, blue);
    leds2[i] = CRGB(red, green, blue);
  }
  FastLED.show();
}

void processCarMovement(String inputValue) {
  // Limit the length to 18 characters
  if (inputValue.length() > 18) {
    inputValue = inputValue.substring(0, 18);
  }
  
  // Print the value received
  Serial.printf("Got value: %s\n", inputValue.c_str());

  // Check if the data starts with "rgb"
  if (inputValue.startsWith("rgb")) {
    // Extract RGB values
    int red, green, blue;
    sscanf(inputValue.c_str(), "rgb,%d,%d,%d", &red, &green, &blue);

    // Set the color of the LEDs
    setLEDs(red, green, blue);
  } else {
    // Send the value via UART if it's not RGB
    sendUARTData(inputValue.c_str());
  }
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      ws.text(client->id(), String("connected"));
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      processCarMovement("0");
      break;
    case WS_EVT_DATA:
      ws.text(client->id(), String("Data Received."));
      AwsFrameInfo *info;
      info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        std::string myData = "";
        myData.assign((char *)data, len);
        processCarMovement(myData.c_str());
      }
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
    default:
      break;
  }
}

void setup(void) {
  Serial.begin(115200);
  
  // Setup UART
  setupUART();
  
  delay(500);
  // Initialize LEDs
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812, LED_2_PIN, GRB>(leds2, NUM_LEDS);
  
  delay(500);
  // Set initial LED color
  setLEDs(0, 200, 0); // Initial green color
  delay(2000);
  setLEDs(0, 0, 0); 


  // Setup WiFi AP
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Setup WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("HTTP server started");


}

void loop() {
  ws.cleanupClients();

    // Buffer to hold incoming data
    uint8_t data[1024];
    static String commandBuffer = ""; // Buffer to accumulate received data

    // Check if data is available
    int len = uart_read_bytes(UART_PORT, data, 1024, 20 / portTICK_RATE_MS);

    // If data is received, process it
    if (len > 0) {
        data[len] = '\0';  // Null-terminate the received string

        // Add received data to command buffer
        commandBuffer += (char*)data;

        // Process each command if multiple are received in one transmission
        while (commandBuffer.indexOf('\n') != -1) {
            // Extract the command from the buffer
            command = commandBuffer.substring(0, commandBuffer.indexOf('\n'));
            commandBuffer = commandBuffer.substring(commandBuffer.indexOf('\n') + 1);

            // Print the complete received command
            //Serial.printf("Received from ESP2: %s\n", receivedCommand.c_str());
         
          // Send the received command back to the WebSocket client (the app)
          ws.textAll(command);
        }
    }
    
  // Optional: Add animation or leave it static based on RGB commands
}
