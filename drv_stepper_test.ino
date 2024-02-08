#include <Arduino.h>

#include <AccelStepper.h>
#include <cmath>

#define enable_pin 5
const float wheel_separation = 140.0;  
const float wheel_radius = 35.0;  

int angMotorSpeedLeftStep = 0;
int angMotorSpeedRightStep = 0;

#define right_stepPin 12
#define right_dirPin 13
#define left_stepPin 14
#define left_dirPin 27
#define motorInterfaceType 1
AccelStepper right_stepper = AccelStepper(motorInterfaceType, right_stepPin, right_dirPin);
AccelStepper left_stepper = AccelStepper(motorInterfaceType, left_stepPin, left_dirPin);
void setup()
{  
  //Serial.begin(115200); 
   right_stepper.setMaxSpeed(6000); //Steps per second
   left_stepper.setMaxSpeed(6000);

   
   pinMode(enable_pin, OUTPUT);
   digitalWrite(enable_pin, HIGH);
}

// Function to calculate motor speeds based on robot linear and angular velocities
void calcSpeed(float robotLinear, float robotAngular) {
  // Convert linear and angular velocities to rad/s
  float angMotorSpeedLeft = (robotLinear - robotAngular * wheel_separation / 2) / wheel_radius;
  float angMotorSpeedRight = (robotLinear + robotAngular * wheel_separation / 2) / wheel_radius;

  // Convert rad/s to steps/s and store them in global variables
  angMotorSpeedLeftStep = angMotorSpeedLeft * 1600 / (2 * 3.14);
  angMotorSpeedRightStep = angMotorSpeedRight * 1600 / (2 * 3.14);
}

// speed in mm/s
bool Straight(int Speed, int Distance, int Dir) {
  
  calcSpeed(Speed, 0);
  //Serial.println(angMotorSpeedLeftStep, angMotorSpeedRightStep);

  // Set speed and direction for both steppers
  int dis = Distance*1600/(3.14*wheel_radius*2);
  
  right_stepper.move(-dis * Dir); //- for making it right direction
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
bool Rotate(int Speed,int Degrees, int Dir) {
   calcSpeed(0, Speed * 3.14/180); //Converting deg/s to rad/s

   float desired_angle_rad = Degrees * 3.14/180;
   float wheel_angle = wheel_separation *  desired_angle_rad/ ( 2 * wheel_radius); // radians
   int wheel_angle_steps = wheel_angle * 1600 / (2 * 3.14);
   
  right_stepper.move(-wheel_angle_steps * Dir); //- for making it right direction
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
bool Circle(int Speed, int Diameter,int Degrees, int Dir) {
  float circle_radius = Diameter/2;
  float Angular_speed = Speed/circle_radius;  //rad
  calcSpeed(Speed, Angular_speed);

  float left_wheel_angle = (Diameter/2 - wheel_separation/2) * (Degrees*3.14/180)/wheel_radius;
  int left_wheel_steps  = left_wheel_angle * 1600 / (2 * 3.14);
   
  float right_wheel_angle = (Diameter/2 + wheel_separation/2) * (Degrees*3.14/180)/wheel_radius;
  int right_wheel_steps  = right_wheel_angle * 1600 / (2 * 3.14);


  right_stepper.move(-right_wheel_steps * Dir); //- for making it right direction
  right_stepper.setSpeed(angMotorSpeedRightStep);

  left_stepper.move(left_wheel_steps * Dir);
  left_stepper.setSpeed(angMotorSpeedLeftStep);
  

  while (right_stepper.distanceToGo() != 0 || left_stepper.distanceToGo() != 0) {
    right_stepper.runSpeedToPosition();
    left_stepper.runSpeedToPosition();
    //right_stepper.runSpeed();
    //left_stepper.runSpeed();
  }

  return true;
}

void loop()
{  

Circle(100, 300, 90, 1);
delay(5000);

}
