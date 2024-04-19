#include <Arduino.h>
#include <AccelStepper.h>
#include <cmath>
#include <HardwareSerial.h>

HardwareSerial SerialBT(1); // UART port for ESP32


// Global variable to store the received line
String receivedLine;
void command_handler(String line);


#define enable_pin 5
const float wheel_separation = 137.5;  
const float wheel_radius = 35.5;  

int angMotorSpeedLeftStep = 0;
int angMotorSpeedRightStep = 0;

#define CCW true            //default turn direction
#define CW false            //opposite turn direction

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
  angMotorSpeedLeftStep = angMotorSpeedLeft * 800 / (2 * PI);
  angMotorSpeedRightStep = angMotorSpeedRight * 800 / (2 * PI);
}

void remote_control(float lin, float ang){
  calcSpeed(lin, -ang);
left_stepper.setSpeed(angMotorSpeedLeftStep);
right_stepper.setSpeed(-angMotorSpeedRightStep);
left_stepper.runSpeed();
right_stepper.runSpeed();

}

void command_handler(String line) {
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

  Serial.println("Command: " + command);
  Serial.println("Arg1: " + arg1);
  Serial.println("Arg2: " + arg2);
   
  // Check if the command is "rc" and call remote_control if it is
  if (command == "rc") {
    remote_control(arg1.toFloat(), arg2.toFloat());
  }
}


void setup()
{  
 Serial.begin(9600);
SerialBT.begin(9600, SERIAL_8N1, 1, 3); // UART config: baudrate, mode, TX pin, RX pin

  // Set up stepper motors
  right_stepper.setMaxSpeed(10000);
  left_stepper.setMaxSpeed(10000);
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, HIGH);


}

void loop()
{  
    // Read data from UART
  if (SerialBT.available()) {
    String receivedLine = SerialBT.readStringUntil('\n');
    //Serial.println("Received: " + receivedLine);

  }
//command_handler(receivedLine);
remote_control(100, 10);
}
