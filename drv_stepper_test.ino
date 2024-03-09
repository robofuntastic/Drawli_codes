#include <Arduino.h>

#include <AccelStepper.h>
#include <cmath>

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
bool Straight(int Speed, float Distance, int Dir) {
  
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
bool Circle(int Speed, int Diameter,int Degrees, int f_b, int Dir) {
  float circle_radius = Diameter/2;
  float Angular_speed = Speed/circle_radius;  //rad
  calcSpeed(Speed, Angular_speed);

  float left_wheel_angle = (Diameter/2 - wheel_separation/2) * (Degrees*3.14/180)/wheel_radius;
  int left_wheel_steps  = left_wheel_angle * 1600 / (2 * 3.14);
   
  float right_wheel_angle = (Diameter/2 + wheel_separation/2) * (Degrees*3.14/180)/wheel_radius;
  int right_wheel_steps  = right_wheel_angle * 1600 / (2 * 3.14);

if (Dir == 1) //cw direction
{
    right_stepper.move(-right_wheel_steps * f_b); //- for making it right direction
  right_stepper.setSpeed(angMotorSpeedRightStep);

  left_stepper.move(left_wheel_steps * f_b);
  left_stepper.setSpeed(angMotorSpeedLeftStep);
}
else{ //ccw direction
  right_stepper.move(-left_wheel_steps * f_b); //- for making it right direction
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

void move(float Distance){
Straight(100, Distance, 1);
}

/***************************************************************************
 ROTATE
 Align robot with bearing. 
 Note ... since the motors are both inwards pointing the wheels require
 the same patterns if they are to rotate in opposite directions.
 ***************************************************************************/
void rotate(float angle, bool turn_ccw){
  int dir = 1;
 	//--------------------
	//take smallest turn
	//--------------------
  	if (angle > PI){				//is the interior angle smaller?
 		angle = 2*PI - angle;
 		turn_ccw = !turn_ccw;
 	}
  if (turn_ccw == false){dir = -1;}
	Rotate(25, angle*180/PI, dir);

}

/***************************************************************************
 MOVE_TO
 Moves robot to next X-Y co-ordinate.  Calculates the distance (steps) and 
 a bearing (radians) from its current co-ordinate. The robot always aligns 
 itself with the new bearing before moving.
 ***************************************************************************/
void move_to(float x2, float y2){
	
	//----------------------------------------------------------
	// static values (contents remain between function calls)
	//----------------------------------------------------------
	static float x1, y1 = 0;			//intial co-ordinates
	static float old_bearing = 0;		//current robot bearing from 3 o'clock

	//----------------------------
	// calculate distance (steps)
	//----------------------------
	float dx = x2 - x1;
	float dy = y2 - y1;
	float distance = sqrt(dx*dx + dy*dy);		//steps (pythagoras)

    //----------------------------------
	// calculate true bearing (radians)
 	//----------------------------------
 	int quadrant;
 	float new_bearing;							//new bearing
 	
 	if ((dx==0) & (dy==0)){quadrant = 0;}		//no change
 	if ((dx>0) & (dy>=0)){quadrant = 1;}
 	if ((dx<=0) & (dy>0)){quadrant = 2;}
 	if ((dx<0) & (dy<=0)){quadrant = 3;}
 	if ((dx>=0) & (dy<0)){quadrant = 4;}
    switch (quadrant){
    	case 0: {new_bearing = 0; break;}
    	case 1: {new_bearing = 0 + asin(dy/distance); break;}
    	case 2: {new_bearing = PI/2 + asin(-dx/distance); break;}
    	case 3: {new_bearing = PI + asin(-dy/distance); break;}
    	case 4: {new_bearing = 2*PI - asin(-dy/distance); break;}
    	default: {break;}
    }

    //----------------------------------------------------------
    // align robot with next bearing.
    //----------------------------------------------------------
 	if (new_bearing < old_bearing){
 		rotate(old_bearing - new_bearing, CW);
 	} else {
 		rotate(new_bearing - old_bearing, CCW);
 	}

 	//------------------------
	// move robot along axis
	//------------------------
	move(distance);								//move the robot

	//------------------------
	// update the static values	
	//------------------------
	x1 = x2;
	y1 = y2;
	old_bearing = new_bearing;   
}




void loop()
{  
// Straight(100, 400, 1);
// delay(2000);
// Rotate(25, 180, -1);
// delay(2000);
// Circle(100, 300, 90, 1, 1);
// delay(2000);
// Circle(50, 200, 90, 1, -1);
// delay(5000);

	// circle ------------
	move_to(136.738441, 145.187821);
	move_to(134.380298, 133.732203);
	move_to(127.595170, 123.920361);
	move_to(117.521703, 117.417222);
	move_to(105.521361, 115.111091);
	move_to(93.521020, 117.417222);
	move_to(83.447553, 123.920361);
	move_to(76.662425, 133.732203);
	move_to(74.304282, 145.187821);
	move_to(76.662425, 156.643438);
	move_to(83.447553, 166.455281);
	move_to(93.521020, 172.958420);
	move_to(105.521361, 175.264551);
	move_to(117.521703, 172.958420);
	move_to(127.595170, 166.455281);
	move_to(134.380298, 156.643438);
	move_to(136.738441, 145.187821);
	move_to(136.738441, 145.187821);

	// back-slash -----------
	move_to(37.813081, 210.330315);

	move_to(174.084903, 79.190066);

	// slash -------------
	move_to(37.527994, 79.190066); 
	move_to(173.799816, 210.330315);

	// square ------------
	move_to(37.656509, 210.457146);
	move_to(173.929525, 210.457146);
	move_to(173.929525, 79.022220);
	move_to(37.656509, 79.022220);
	move_to(37.656509, 210.457146);

	// home --------------
	move_to(0.0000, 0.0000);
}
