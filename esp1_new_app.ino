#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "driver/uart.h"
#include <FastLED.h>
#include <cmath>

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

int prevRed = -1, prevGreen = -1, prevBlue = -1;  // Variables to store previous RGB values

const float wheel_separation = 116.1;  //last 115.70
const float wheel_radius = 38.5;  //38.7
const int microstepping = 8;  
int angMotorSpeedLeftStep = 0;
int angMotorSpeedRightStep = 0;

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

void calcSpeed(float robotLinear, float robotAngular) {
  // Convert linear and angular velocities to rad/s
  float angMotorSpeedLeft = (robotLinear - robotAngular * wheel_separation / 2) / wheel_radius;
  float angMotorSpeedRight = (robotLinear + robotAngular * wheel_separation / 2) / wheel_radius;

  // Convert rad/s to steps/s and store them in global variables
  angMotorSpeedLeftStep = angMotorSpeedLeft * 200*microstepping / (2 * PI);
  angMotorSpeedRightStep = angMotorSpeedRight * 200*microstepping / (2 * PI);
}


#define ANIMATION_INTERVAL 50  // Time between each animation step (in milliseconds)
unsigned long previousMillis = 0;
bool isAnimating = false;
int animationStep = 0;  // To track the current animation step
int sameCommandCount = 0;  // To track how many times the same RGB command is received

// 5 Different LED Animations
void animateLEDs(int preset) {
  unsigned long currentMillis = millis();
int red =  200;
int green =  200;
int blue =200;
  // Check if it's time to update the animation
  if (currentMillis - previousMillis >= ANIMATION_INTERVAL) {
    previousMillis = currentMillis;  // Update the time marker

    // Select an animation based on the number of times the same command is received
    switch (preset) {
      case 1:
        // Animation 1: Simple fade in and out
        for (int i = 0; i < NUM_LEDS; i++) {
          int brightness = 128 + 127 * sin(animationStep * 0.1);
          leds[i] = CRGB(red * brightness / 255, green * brightness / 255, blue * brightness / 255);
          leds2[i] = CRGB(red * brightness / 255, green * brightness / 255, blue * brightness / 255);
        }
        break;
      case 2:
        // Animation 2: Blinking LEDs
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = (animationStep % 20 < 10) ? CRGB(red, green, blue) : CRGB(0, 0, 0);
          leds2[i] = (animationStep % 20 < 10) ? CRGB(red, green, blue) : CRGB(0, 0, 0);
        }
        break;
      case 3:
        // Animation 3: Color wipe
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = (i <= animationStep % NUM_LEDS) ? CRGB(red, green, blue) : CRGB(0, 0, 0);
          leds2[i] = (i <= animationStep % NUM_LEDS) ? CRGB(red, green, blue) : CRGB(0, 0, 0);
        }
        break;
      case 4:
        // Animation 4: Rainbow cycling
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CHSV((animationStep + i * 10) % 255, 255, 255);  // Hue cycling
          leds2[i] = CHSV((animationStep + i * 10) % 255, 255, 255);
        }
        break;
      case 5:
        // Animation 5: Random sparkle
        for (int i = 0; i < NUM_LEDS; i++) {
          if (random(10) > 7) {
            leds[i] = CRGB(red, green, blue);
            leds2[i] = CRGB(red, green, blue);
          } else {
            leds[i] = CRGB(0, 0, 0);
            leds2[i] = CRGB(0, 0, 0);
          }
        }
        break;
    }
    FastLED.show();
    animationStep++;  // Move to the next step of the animation
    if (animationStep > 100) animationStep = 0;  // Reset the animation step periodically
  }
}

// Modify processCarMovement to handle repeated commands and trigger animations
int animation_preset = 0;

void processCarMovement(String inputValue) {
  if (inputValue.length() > 22) {
    inputValue = inputValue.substring(0, 22);
  }

  Serial.printf("Got value: %s\n", inputValue.c_str());

  if (inputValue.startsWith("rgb")) {
    int red, green, blue;
    sscanf(inputValue.c_str(), "rgb,%d,%d,%d", &red, &green, &blue);

      setLEDs(red, green, blue);  // Static color change
      isAnimating = false;  // Stop animation
 
  } 
   else if (inputValue.startsWith("preset1")) {
    animation_preset = 1;
      isAnimating = true; 
  }
   else if (inputValue.startsWith("preset2")) {
    animation_preset = 2;
      isAnimating = true; 
  }
   else if (inputValue.startsWith("preset3")) {
    animation_preset = 3;
      isAnimating = true; 
  }
   else if (inputValue.startsWith("preset4")) {
    animation_preset = 4;
      isAnimating = true; 
  }
   else if (inputValue.startsWith("preset5")) {
    animation_preset = 5;
      isAnimating = true; 
  }
   else if (inputValue.startsWith("rc")) {
    sendUARTData(inputValue.c_str());
    animation_preset = 4;
      isAnimating = true; 


  }
  else {
    sendUARTData(inputValue.c_str());
  }
}

// Non-blocking LED setting
void setLEDs(int red, int green, int blue) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(red, green, blue);
    leds2[i] = CRGB(red, green, blue);
  }
  FastLED.show();
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
    
  // Check if the animation should run
  if (isAnimating) {
    // Animate with the last received RGB color
    animateLEDs(animation_preset);
  }

}
