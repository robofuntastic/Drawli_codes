
#include <AccelStepper.h>

#define enable_pin 5

#define stepPin 12
#define dirPin 13
#define stepPin_2 14
#define dirPin_2 27
#define motorInterfaceType 1
//AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
AccelStepper stepper_2 = AccelStepper(motorInterfaceType, stepPin_2, dirPin_2);
void setup()
{  
   stepper.setMaxSpeed(1600);
   stepper.setSpeed(50);	
   stepper_2.setMaxSpeed(1600);
   stepper_2.setSpeed(50); 
   pinMode(enable_pin, OUTPUT);
   digitalWrite(enable_pin, HIGH);
}

void loop()
{  
   stepper.runSpeed();
   stepper_2.runSpeed();
}
