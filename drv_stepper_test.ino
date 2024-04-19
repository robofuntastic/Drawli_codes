#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const char *ssid = "RoboFuntastic";
const char *password = "123456789";

// Global variable to store the received line
String receivedLine;
void command_handler(String line);

WebServer server(80);

#define enable_pin 5
const float wheel_separation = 137.5;  
const float wheel_radius = 35.5;  

int angMotorSpeedLeftStep = 0;
int angMotorSpeedRightStep = 0;

#define CCW true						//default turn direction
#define CW false						//opposite turn direction

#define right_stepPin 12
#define right_dirPin 13
#define left_stepPin 14
#define left_dirPin 27
#define motorInterfaceType 1
AccelStepper right_stepper = AccelStepper(motorInterfaceType, right_stepPin, right_dirPin);
AccelStepper left_stepper = AccelStepper(motorInterfaceType, left_stepPin, left_dirPin);

// Function to calculate motor speeds based on robot linear and angular velocities
//units mm/s and rad/s
void calcSpeed(float robotLinear, float robotAngular) {
  // Convert linear and angular velocities to rad/s
  float angMotorSpeedLeft = (robotLinear - robotAngular * wheel_separation / 2) / wheel_radius;
  float angMotorSpeedRight = (robotLinear + robotAngular * wheel_separation / 2) / wheel_radius;

  // Convert rad/s to steps/s and store them in global variables
  angMotorSpeedLeftStep = angMotorSpeedLeft * 1600 / (2 * PI);
  angMotorSpeedRightStep = angMotorSpeedRight * 1600 / (2 * PI);
}

void remote_control(float lin, float ang){
  calcSpeed(lin*1000, -ang);
left_stepper.setSpeed(angMotorSpeedLeftStep);
right_stepper.setSpeed(-angMotorSpeedRightStep);
left_stepper.runSpeed();
right_stepper.runSpeed();

}

void command_handler(String line)
{
  // Extract command and arguments
  int index = line.indexOf(',');
  String command = line.substring(0, index);
  String args = line.substring(index + 1);

  // Parse arguments
  int index2 = args.indexOf(',');
  String arg1 = args.substring(0, index2);

  // Skip over the comma and get the next argument
  args = args.substring(index2 + 1);
  index2 = args.indexOf(',');
  String arg2 = args.substring(0, index2);

  // Debug prints
  //Serial.println("Command: " + command);
  //Serial.println("Arg1: " + arg1);
  // Serial.println("Arg2: " + arg2);

  // Check if the command is "rc" and call remote_control if it is
  if (command == "rc") {
    remote_control(arg1.toFloat(), arg2.toFloat());
  }
}


// Task function for handling Wi-Fi and client connections
void wifi_task(void *pvParameters) {
  while (true) {
    WiFiClient client = server.client();
    if (client.connected()) {
      Serial.println("Client connected");
      while (client.connected()) {
        if (client.available()) {
          String receivedLine = client.readStringUntil('\r');
          Serial.println("Received from client: " + receivedLine);
          // Handle the received command
          command_handler(receivedLine);
        }
      }
      client.stop();
      Serial.println("Client disconnected");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay for 100ms
  }
}

void setup()
{  
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.softAP(ssid, password, 1, 0);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Set up stepper motors
  right_stepper.setMaxSpeed(10000);
  left_stepper.setMaxSpeed(10000);
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, HIGH);

  // Start server
  server.begin();
  Serial.println("HTTP server started");

  // Create a task to handle Wi-Fi connections
  xTaskCreatePinnedToCore(
    wifi_task,
    "wifi_task",
    10000,  // Stack size
    NULL,
    configMAX_PRIORITIES - 1,  // Priority
    NULL,
    1  // Core to run the task on (Core 0)
  );
}

void loop()
{  
  // Handle client requests in main loop
  server.handleClient();
}
