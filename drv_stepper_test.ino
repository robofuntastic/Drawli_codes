
#include <AccelStepper.h>

#define enable_pin 5

#define right_stepPin 12
#define right_dirPin 13
#define left_stepPin 14
#define left_dirPin 27
#define motorInterfaceType 1
AccelStepper right_stepper = AccelStepper(motorInterfaceType, right_stepPin, right_dirPin);
AccelStepper left_stepper = AccelStepper(motorInterfaceType, left_stepPin, left_dirPin);
void setup()
{  
   right_stepper.setMaxSpeed(1600); //Steps per second
   right_stepper.setSpeed(-400);	//Steps per second
   left_stepper.setMaxSpeed(1600);
   left_stepper.setSpeed(400); 
   pinMode(enable_pin, OUTPUT);
   digitalWrite(enable_pin, HIGH);
}

void loop()
{  
   right_stepper.runSpeed();
   left_stepper.runSpeed();
}
