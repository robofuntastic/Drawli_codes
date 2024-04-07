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

#define CCW true  // default turn direction
#define CW false  // opposite turn direction

#define right_stepPin 12
#define right_dirPin 13
#define left_stepPin 14
#define left_dirPin 27
#define motorInterfaceType 1
AccelStepper right_stepper = AccelStepper(motorInterfaceType, right_stepPin, right_dirPin);
AccelStepper left_stepper = AccelStepper(motorInterfaceType, left_stepPin, left_dirPin);

// Function to calculate motor speeds based on robot linear and angular velocities
// units mm/s and rad/s
void calcSpeed(float robotLinear, float robotAngular) {
  // Convert linear and angular velocities to rad/s
  float angMotorSpeedLeft = (robotLinear - robotAngular * wheel_separation / 2) / wheel_radius;
  float angMotorSpeedRight = (robotLinear + robotAngular * wheel_separation / 2) / wheel_radius;

  // Convert rad/s to steps/s and store them in global variables
  angMotorSpeedLeftStep = angMotorSpeedLeft * 1600 / (2 * PI);
  angMotorSpeedRightStep = angMotorSpeedRight * 1600 / (2 * PI);
}

// speed in mm/s
bool Straight(int Speed, float Distance, int Dir) {

  calcSpeed(Speed, 0);
  // Serial.println(angMotorSpeedLeftStep, angMotorSpeedRightStep);

  // Set speed and direction for both steppers
  int dis = Distance * 1600 / (PI * wheel_radius * 2);

  right_stepper.move(-dis * Dir);  //- for making it right direction
  right_stepper.setSpeed(angMotorSpeedRightStep);

  left_stepper.move(dis * Dir);
  left_stepper.setSpeed(angMotorSpeedLeftStep);

  while (right_stepper.distanceToGo() != 0 || left_stepper.distanceToGo() != 0) {
    right_stepper.runSpeedToPosition();
    left_stepper.runSpeedToPosition();
  }
  return true;
}

// speed degrees/s
bool Rotate(int Speed, int Degrees, int Dir) {
  calcSpeed(0, Speed * PI / 180);  // Converting deg/s to rad/s

  float desired_angle_rad = Degrees * PI / 180;
  float wheel_angle = wheel_separation * desired_angle_rad / (2 * wheel_radius);  // radians
  int wheel_angle_steps = wheel_angle * 1600 / (2 * PI);

  right_stepper.move(-wheel_angle_steps * Dir);  //- for making it right direction
  right_stepper.setSpeed(angMotorSpeedRightStep);

  left_stepper.move(-wheel_angle_steps * Dir);
  left_stepper.setSpeed(angMotorSpeedLeftStep);

  while (right_stepper.distanceToGo() != 0 || left_stepper.distanceToGo() != 0) {
    right_stepper.runSpeedToPosition();
    left_stepper.runSpeedToPosition();
  }

  return true;
}

// mm/s , mm, degree
bool Circle(int Speed, int Diameter, int Degrees, int f_b, int Dir) {
  float circle_radius = Diameter / 2;
  float Angular_speed = Speed / circle_radius;  // rad
  calcSpeed(Speed, Angular_speed);

  float left_wheel_angle = (Diameter / 2 - wheel_separation / 2) * (Degrees * PI / 180) / wheel_radius;
  int left_wheel_steps = left_wheel_angle * 1600 / (2 * PI);

  float right_wheel_angle = (Diameter / 2 + wheel_separation / 2) * (Degrees * PI / 180) / wheel_radius;
  int right_wheel_steps = right_wheel_angle * 1600 / (2 * PI);

  if (Dir == 1)  // cw direction
  {
    right_stepper.move(-right_wheel_steps * f_b);  //- for making it right direction
    right_stepper.setSpeed(angMotorSpeedRightStep);

    left_stepper.move(left_wheel_steps * f_b);
    left_stepper.setSpeed(angMotorSpeedLeftStep);
  } else {                                        // ccw direction
    right_stepper.move(-left_wheel_steps * f_b);  //- for making it right direction
    right_stepper.setSpeed(angMotorSpeedLeftStep);

    left_stepper.move(right_wheel_steps * f_b);
    left_stepper.setSpeed(angMotorSpeedRightStep);
  }

  while (right_stepper.distanceToGo() != 0 || left_stepper.distanceToGo() != 0) {
    right_stepper.runSpeedToPosition();
    left_stepper.runSpeedToPosition();
  }

  return true;
}

void move(float Distance) {
  Straight(100, Distance, 1);
}

/***************************************************************************
 ROTATE
 Align robot with bearing.
 Note ... since the motors are both inwards pointing the wheels require
 the same patterns if they are to rotate in opposite directions.
 ***************************************************************************/
void rotate(float angle, bool turn_ccw) {
  int dir = 1;
  //--------------------
  // take smallest turn
  //--------------------
  if (angle > PI) {  // is the interior angle smaller?
    angle = 2 * PI - angle;
    turn_ccw = !turn_ccw;
  }
  if (turn_ccw == false) {
    dir = -1;
  }
  Rotate(25, angle * 180 / PI, dir);
}

/***************************************************************************
 MOVE_TO
 Moves robot to next X-Y co-ordinate.  Calculates the distance (steps) and
 a bearing (radians) from its current co-ordinate. The robot always aligns
 itself with the new bearing before moving.
 ***************************************************************************/
void move_to(float x2, float y2) {

  //----------------------------------------------------------
  // static values (contents remain between function calls)
  //----------------------------------------------------------
  static float x1, y1 = 0;       // intial co-ordinates
  static float old_bearing = 0;  // current robot bearing from 3 o'clock

  //----------------------------
  // calculate distance (steps)
  //----------------------------
  float dx = x2 - x1;
  float dy = y2 - y1;
  float distance = sqrt(dx * dx + dy * dy);  // steps (pythagoras)

  //----------------------------------
  // calculate true bearing (radians)
  //----------------------------------
  int quadrant;
  float new_bearing;  // new bearing

  if ((dx == 0) & (dy == 0)) {
    quadrant = 0;
  }  // no change
  if ((dx > 0) & (dy >= 0)) {
    quadrant = 1;
  }
  if ((dx <= 0) & (dy > 0)) {
    quadrant = 2;
  }
  if ((dx < 0) & (dy <= 0)) {
    quadrant = 3;
  }
  if ((dx >= 0) & (dy < 0)) {
    quadrant = 4;
  }
  switch (quadrant) {
    case 0:
      {
        new_bearing = 0;
        break;
      }
    case 1:
      {
        new_bearing = 0 + asin(dy / distance);
        break;
      }
    case 2:
      {
        new_bearing = PI / 2 + asin(-dx / distance);
        break;
      }
    case 3:
      {
        new_bearing = PI + asin(-dy / distance);
        break;
      }
    case 4:
      {
        new_bearing = 2 * PI - asin(-dy / distance);
        break;
      }
    default:
      {
        break;
      }
  }

  //----------------------------------------------------------
  // align robot with next bearing.
  //----------------------------------------------------------
  if (new_bearing < old_bearing) {
    rotate(old_bearing - new_bearing, CW);
  } else {
    rotate(new_bearing - old_bearing, CCW);
  }

  //------------------------
  // move robot along axis
  //------------------------
  move(distance);  // move the robot

  //------------------------
  // update the static values
  //------------------------
  x1 = x2;
  y1 = y2;
  old_bearing = new_bearing;
}

void remote_control(float lin, float ang) {
  calcSpeed(lin * 1000, -ang);
  left_stepper.setSpeed(angMotorSpeedLeftStep);
  right_stepper.setSpeed(-angMotorSpeedRightStep);
  left_stepper.runSpeed();
  right_stepper.runSpeed();
  // Print the values of angMotorSpeedLeftStep and angMotorSpeedRightStep
  // Serial.print("Left Speed: ");
  // Serial.println(angMotorSpeedLeftStep);
  // Serial.print("Right Speed: ");
  // Serial.println(angMotorSpeedRightStep);
}

String args = "0", arg1 = "0", arg2 = "0", command = "";

void command_handler(String line) {
  // Extract command and arguments
  int index = line.indexOf(',');
  command = line.substring(0, index);
  args = line.substring(index + 1);

  // Parse arguments
  int index2 = args.indexOf(',');
  arg1 = args.substring(0, index2);

  // Skip over the comma and get the next argument
  args = args.substring(index2 + 1);
  index2 = args.indexOf(',');
  arg2 = args.substring(0, index2);

  // Debug prints
  Serial.println("Command: " + command);
  Serial.println("Arg1: " + arg1);
  Serial.println("Arg2: " + arg2);

  // Check if the command is "rc" and call remote_control if it is
}

void execute_cmd() {

  // if (command == "rc") {
  //   // remote_control(arg1.toFloat(), arg2.toFloat());
  // }
    remote_control(0.4, 0.3);
}

// Task function for running parrallel
void parrallel_tasks(void *pvParameters) {
  while (true) {
    execute_cmd();
    // vTaskDelay(1 / portTICK_PERIOD_MS);  // Delay for 1
  }
}

void receive_cmds(void *ptr) {

  // Start server
  server.begin();
  Serial.println("HTTP server started");

  while (1) {
    server.handleClient();
    WiFiClient client = server.client();
    if (client.connected()) {
      Serial.println("Client connected");
      while (client.connected()) {
        if (client.available()) {
          receivedLine = client.readStringUntil('\r');
          // Serial.print("Received from client: ");
          // Serial.println(receivedLine);
          command_handler(receivedLine);

          // client.println("Received from client: ");
          // client.println(line);
          // xSemaphoreGive(xSemaphoreActivateParrallelTasks);
        }
        vTaskDelay(1);
      }
      client.stop();
      Serial.println("Client disconnected");
    }
    vTaskDelay(1);
  }
  // Your additional code for controlling the robot can go here
  // For example, to move forward at speed 50 mm/s for 150 mm:
  // Straight(50, 150, 1);
  // vTaskDelay(1);
}

void setup() {
  Serial.begin(115200);
  /* Attempt to create a semaphore. */
  // xSemaphoreActivateParrallelTasks = xSemaphoreCreateBinary();

  // Connect to Wi-Fi
  // WiFi.softAP(ssid, password, 1, 0);
  // IPAddress IP = WiFi.softAPIP();
  // Serial.print("AP IP address: ");
  // Serial.println(IP);

  Serial.println(getCpuFrequencyMhz());
  setCpuFrequencyMhz(240);
  // Serial.println(ESP.getFlashChipSpeed());
  Serial.println(getCpuFrequencyMhz());
  // server.begin();
  // Serial.println("HTTP server started");

  // Set up stepper motors
  right_stepper.setMaxSpeed(10000);
  left_stepper.setMaxSpeed(10000);
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, HIGH);

  // Create a task to run the Straight function
  // xTaskCreatePinnedToCore(
  //   receive_cmds,
  //   "receive_cmds",
  //   1024 * 10,  // Stack size
  //   NULL,
  //   1,  // Priority
  //   NULL,
  //   0  // Core to run the task on (Core 0)
  // );

  xTaskCreatePinnedToCore(
    parrallel_tasks,
    "parrallel_tasks",
    10000,  // Stack size
    NULL,
    1,  // Priority
    NULL,
    1  // Core to run the task on (Core 0)
  );
}
void loop() {
}