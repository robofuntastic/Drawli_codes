#include <Arduino.h>
#include "driver/uart.h"
#include <AccelStepper.h>
#include <cmath>
#include <ESP32Servo.h>


#define TX_PIN 16
#define RX_PIN 17
#define UART_PORT UART_NUM_1
#define BUF_SIZE (1024)

Servo myservo;  // create servo object to control a servo

#define BATTERY_PIN 36  // Battery voltage measurement pin
#define R1 10000.0      // Resistor R1 value in ohms (10k)
#define R2 2000.0       // Resistor R2 value in ohms (2k)
#define MAX_VOLTAGE 16.25 // Maximum battery voltage
#define MIN_VOLTAGE 13.40 // Minimum battery voltage

unsigned long previousMillisBattery = 0;
const unsigned long batteryInterval = 10000; // 1 minute interval

// Global variable to store the received line
String command;
void command_handler(String line);

#define TX_PIN 16 // GPIO1
#define RX_PIN 17 // GPIO3


const float wheel_separation = 116.1;  //last 115.70
const float wheel_radius = 38.5;  //38.7

const int microstepping = 8;  

int angMotorSpeedLeftStep = 0;
int angMotorSpeedRightStep = 0;

// Define global variables to store previous X and Y values
float previousXValue = 0.0;
float previousYValue = 0.0;

#define CCW true            //default turn direction
#define CW false            //opposite turn direction

#define right_stepPin 18
#define right_dirPin 4
#define right_enPin 22
#define right_MS1 21
#define right_MS2 19


#define left_stepPin 26
#define left_dirPin 25
#define left_enPin 12
#define left_MS1 14
#define left_MS2 27

int servoPin = 23;

#define motorInterfaceType 1
AccelStepper right_stepper = AccelStepper(motorInterfaceType, right_stepPin, right_dirPin);
AccelStepper left_stepper = AccelStepper(motorInterfaceType, left_stepPin, left_dirPin);

void send_to_esp1(String message) {
    // Convert the message to a C-style string
    const char* data = message.c_str();
    
    // Send the message to ESP1 over UART
    int len = uart_write_bytes(UART_PORT, data, strlen(data));
    
    // Optionally add a newline character to indicate end of the message
    uart_write_bytes(UART_PORT, "\n", 1);
    
    // Print the message to Serial for debugging
    Serial.printf("Sent to ESP1: %s\n", message.c_str());
}


// Function to calculate motor speeds based on robot linear and angular velocities
//units mm/s and rad/s
void calcSpeed(float robotLinear, float robotAngular) {
  // Convert linear and angular velocities to rad/s
  float angMotorSpeedLeft = (robotLinear - robotAngular * wheel_separation / 2) / wheel_radius;
  float angMotorSpeedRight = (robotLinear + robotAngular * wheel_separation / 2) / wheel_radius;

  // Convert rad/s to steps/s and store them in global variables
  angMotorSpeedLeftStep = angMotorSpeedLeft * 200*microstepping / (2 * PI);
  angMotorSpeedRightStep = angMotorSpeedRight * 200*microstepping / (2 * PI);
  //Serial.println(  angMotorSpeedLeft);
}
// speed in mm/s
void Straight(int Speed, int Distance, int pen) {

    if (pen == 0) {
      myservo.write(0);
    } else if (pen == 1) {
      myservo.write(180);
    }
    
  calcSpeed(Speed, 0);
  // Serial.println(angMotorSpeedLeftStep, angMotorSpeedRightStep);

  // Set speed and direction for both steppers
  int dis = Distance * 200*microstepping / (PI * wheel_radius * 2);

  right_stepper.move(dis);  //- for making it right direction
  right_stepper.setSpeed(angMotorSpeedRightStep);

  left_stepper.move(-dis);
  left_stepper.setSpeed(angMotorSpeedLeftStep);

  while (right_stepper.distanceToGo() != 0 || left_stepper.distanceToGo() != 0) {
    right_stepper.runSpeedToPosition();
    left_stepper.runSpeedToPosition();
  }
  command = "st";
  send_to_esp1("str_ok");
}

// speed degrees/s
void Rotate(int Degrees, int  Speed, int pen) {
        if (pen == 0) {
      myservo.write(0);
    } else if (pen == 1) {
      myservo.write(180);
    }
    
  calcSpeed(0, Speed * PI / 180);  // Converting deg/s to rad/s

  float desired_angle_rad = Degrees * PI / 180;
  float wheel_angle = wheel_separation * desired_angle_rad / (2 * wheel_radius);  // radians
  int wheel_angle_steps = wheel_angle * 200*microstepping / (2 * PI);

  right_stepper.move(wheel_angle_steps ); 
  right_stepper.setSpeed(angMotorSpeedRightStep);

  left_stepper.move(wheel_angle_steps );
  left_stepper.setSpeed(angMotorSpeedLeftStep);

  while (right_stepper.distanceToGo() != 0 || left_stepper.distanceToGo() != 0) {
    right_stepper.runSpeedToPosition();
    left_stepper.runSpeedToPosition();
  }

  command = "st";
  send_to_esp1("rot_ok");
}

// mm/s , mm, degree
void Circle( int Diameter, int Degrees, int Speed, int pen) {
      if (pen == 0) {
      myservo.write(0);
    } else if (pen == 1) {
      myservo.write(180);
    }
    
  float circle_radius = Diameter / 2;
  float Angular_speed = Speed / circle_radius;  // rad
  calcSpeed(Speed, Angular_speed);

  float left_wheel_angle = (Diameter / 2 - wheel_separation / 2) * (Degrees * PI / 180) / wheel_radius;
  int left_wheel_steps = left_wheel_angle * 200*microstepping / (2 * PI);

  float right_wheel_angle = (Diameter / 2 + wheel_separation / 2) * (Degrees * PI / 180) / wheel_radius;
  int right_wheel_steps = right_wheel_angle * 200*microstepping / (2 * PI);


  if (Degrees > 0)  // ccw direction
  {
    right_stepper.move(right_wheel_steps);  
    right_stepper.setSpeed(angMotorSpeedRightStep);

    left_stepper.move(-left_wheel_steps);
    left_stepper.setSpeed(angMotorSpeedLeftStep);
  } else {                                        // cw direction
    right_stepper.move(-left_wheel_steps ); 
    right_stepper.setSpeed(angMotorSpeedLeftStep);

    left_stepper.move(right_wheel_steps);
    left_stepper.setSpeed(angMotorSpeedRightStep);
  }

  while (right_stepper.distanceToGo() != 0 || left_stepper.distanceToGo() != 0) {
    right_stepper.runSpeedToPosition();
    left_stepper.runSpeedToPosition();
  }

 command = "st";
 send_to_esp1("cir_ok");
}

void g_move(float Distance,int pen) {
  Straight(50, Distance, pen);
}

void g_rotate(float angle, bool turn_ccw, int pen) {
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
  Rotate( angle * 180 / PI * dir, 20, pen);
}

void g_move_to(int x2, int y2, int z) {

//Serial.printf("X: %d, Y: %d, Z: %d\n", x2, y2, z);

int pen = 0;

    if (z == 0) {
    pen = 1;
    //Serial.printf("pen 1");
  } else {
    pen = 0;
    //Serial.printf("pen 0");
  }

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
    g_rotate(old_bearing - new_bearing, CW, pen);
  } else {
    g_rotate(new_bearing - old_bearing, CCW, pen);
  }

  //------------------------
  // move robot along axis
  //------------------------
  g_move(distance, pen);  // move the robot

  //------------------------
  // update the static values
  //------------------------
  x1 = x2;
  y1 = y2;
  old_bearing = new_bearing;

  command = "st";
  send_to_esp1("g_ok");
}

void remote_control(float lin, float ang){
  calcSpeed(lin, -ang * PI / 180);
  
left_stepper.setSpeed(-angMotorSpeedLeftStep);
right_stepper.setSpeed(angMotorSpeedRightStep);
left_stepper.runSpeed();
right_stepper.runSpeed();

}

// Command handler to handle both motor and servo commands
void command_handler(String line) {
  int index = line.indexOf(',');
  String cmd = line.substring(0, index);
  String args = line.substring(index + 1);

  //Serial.printf("arg1: %s, arg2: %s\n", arg1.c_str(), arg2.c_str());

  if (cmd == "rc") {
    
  int index2 = args.indexOf(',');
  String arg1 = args.substring(0, index2);
  
  args = args.substring(index2 + 1);
  index2 = args.indexOf(',');
  String arg2 = args.substring(0, index2);
  
    // Extract the third argument
  args = args.substring(index2 + 1);
  index2 = args.indexOf(',');
  String arg3 = args.substring(0, index2);
  
    remote_control(arg1.toFloat(), arg2.toFloat());
    
    if (arg3.toFloat() == 0.0) {
      myservo.write(0);
    } else if (arg3.toFloat() == 1.0) {
      myservo.write(180);
    }
    
  }

  else if (cmd == "pa") {
  int index2 = args.indexOf(',');
  String arg1 = args.substring(0, index2);
  
    if (arg1.toFloat() == 1.0) {
      myservo.write(0);
    } else if (arg1.toFloat() == -1.0) {
      myservo.write(180);
    }
  }

  else if (cmd == "str") {
  int index2 = args.indexOf(',');
  String arg1 = args.substring(0, index2);
  
  args = args.substring(index2 + 1);
  index2 = args.indexOf(',');
  String arg2 = args.substring(0, index2);
  
    // Extract the third argument
  args = args.substring(index2 + 1);
  index2 = args.indexOf(',');
  String arg3 = args.substring(0, index2);
  
Straight(arg1.toFloat(),arg2.toFloat(),arg3.toFloat());
}

  else if (cmd == "rot") {
  int index2 = args.indexOf(',');
  String arg1 = args.substring(0, index2);
  
  args = args.substring(index2 + 1);
  index2 = args.indexOf(',');
  String arg2 = args.substring(0, index2);
  
    // Extract the third argument
  args = args.substring(index2 + 1);
  index2 = args.indexOf(',');
  String arg3 = args.substring(0, index2);
  
Rotate(arg1.toFloat(),arg2.toFloat(),arg3.toFloat());
}

 else if (cmd == "cir") {
  int index2 = args.indexOf(',');
  String arg1 = args.substring(0, index2);
  
  args = args.substring(index2 + 1);
  index2 = args.indexOf(',');
  String arg2 = args.substring(0, index2);
  
    // Extract the third argument
  args = args.substring(index2 + 1);
  index2 = args.indexOf(',');
  String arg3 = args.substring(0, index2);
  
  String arg4 = args.substring(index2 + 1);
Circle(arg1.toFloat(),arg2.toFloat(),arg3.toFloat(),arg4.toFloat());
}


  // Handling G01 Command
  else if (line.startsWith("G01")) {

      // Initialize Z value and default X, Y to the previous ones
    float xValue = previousXValue;
    float yValue = previousYValue;
    float zValue = 0.0;

    // Parse G01 command without commas (e.g., "G01 X1130 Y595 Z1000")
    int xIndex = line.indexOf('X');
    int yIndex = line.indexOf('Y');
    int zIndex = line.indexOf('Z');

    // Extract X value (if present), else use previous value
    if (xIndex != -1) {
      int endX = (yIndex != -1) ? yIndex : ((zIndex != -1) ? zIndex : line.length());
      xValue = line.substring(xIndex + 1, endX).toFloat();
      previousXValue = xValue;  // Update previous X value
    }

    // Extract Y value (if present), else use previous value
    if (yIndex != -1) {
      int endY = (zIndex != -1) ? zIndex : line.length();
      yValue = line.substring(yIndex + 1, endY).toFloat();
      previousYValue = yValue;  // Update previous Y value
    }

    // Extract Z value (if present)
    if (zIndex != -1) {
      zValue = line.substring(zIndex + 1).toFloat();
    }
    g_move_to(xValue, yValue, zValue);
    
    
  }

}

// Function to read battery voltage
float readBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  float Vout = (adcValue * 3.3) / 4095.0;  // Convert ADC value to Vout (3.3V reference)
  float Vin = Vout * (R1 + R2) / R2;       // Calculate Vin using voltage divider formula
  return Vin;
}

// Function to calculate battery percentage based on voltage
float calculateBatteryPercentage(float voltage) {
  if (voltage >= MAX_VOLTAGE) {
    return 100.0; // 100% battery if voltage is greater than or equal to max voltage
  }
  if (voltage <= MIN_VOLTAGE) {
    return 0.0;  // 0% battery if voltage is less than or equal to min voltage
  }
  // Map voltage to percentage between MIN_VOLTAGE and MAX_VOLTAGE
  return ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0;
}

void setup() {
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
    uart_driver_install(UART_PORT, BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Initialize serial for debugging
    Serial.begin(115200);
    Serial.println("UART Receiver Ready");

      // Set up stepper motors
  right_stepper.setMaxSpeed(10000);
  left_stepper.setMaxSpeed(10000);
  pinMode(right_enPin, OUTPUT);
  digitalWrite(right_enPin, LOW);
  pinMode(right_MS1, OUTPUT);
  digitalWrite(right_MS1, LOW);
  pinMode(right_MS2, OUTPUT);
  digitalWrite(right_MS2, LOW);

  pinMode(left_enPin, OUTPUT);
  digitalWrite(left_enPin, LOW);
  pinMode(left_MS1, OUTPUT);
  digitalWrite(left_MS1, LOW);
  pinMode(left_MS2, OUTPUT);
  digitalWrite(left_MS2, LOW);

  pinMode(BATTERY_PIN, INPUT);
  
  // Set up servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // Servo initialization
}

void loop() {
    unsigned long currentMillis = millis();
    
  // Check battery voltage every minute
  if (currentMillis - previousMillisBattery >= batteryInterval) {
    previousMillisBattery = currentMillis;  // Update the time marker
    float batteryVoltage = readBatteryVoltage();
    int batteryPercentage = calculateBatteryPercentage(batteryVoltage);
    send_to_esp1("bat," + String(batteryPercentage));
  }
  
    // Buffer to hold incoming data
    uint8_t data[BUF_SIZE];
    static String commandBuffer = ""; // Buffer to accumulate command data

    // Check if data is available
    int len = uart_read_bytes(UART_PORT, data, BUF_SIZE, 0);

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

            // Print the complete command
            Serial.printf("Received command via UART: %s\n", command.c_str());
        }
    }

    // Small delay for better serial output handling
    command_handler(command);

}
