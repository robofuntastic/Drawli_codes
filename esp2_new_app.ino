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

// Global variable to store the received line
String command;
void command_handler(String line);

#define TX_PIN 16 // GPIO1
#define RX_PIN 17 // GPIO3


const float wheel_separation = 118;  
const float wheel_radius = 40.5;  

int angMotorSpeedLeftStep = 0;
int angMotorSpeedRightStep = 0;

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
int batteryPin = 36;

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
  angMotorSpeedLeftStep = angMotorSpeedLeft * 400 / (2 * PI);
  angMotorSpeedRightStep = angMotorSpeedRight * 400 / (2 * PI);
  //Serial.println(  angMotorSpeedLeft);
}
// speed in mm/s
void Straight(int Speed, float Distance, int pen) {

    if (pen == 0) {
      myservo.write(0);
    } else if (pen == 1) {
      myservo.write(180);
    }
    
  calcSpeed(Speed, 0);
  // Serial.println(angMotorSpeedLeftStep, angMotorSpeedRightStep);

  // Set speed and direction for both steppers
  int dis = Distance * 400 / (PI * wheel_radius * 2);

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
  int wheel_angle_steps = wheel_angle * 400 / (2 * PI);

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
  int left_wheel_steps = left_wheel_angle * 400 / (2 * PI);

  float right_wheel_angle = (Diameter / 2 + wheel_separation / 2) * (Degrees * PI / 180) / wheel_radius;
  int right_wheel_steps = right_wheel_angle * 400 / (2 * PI);


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
  digitalWrite(right_MS1, HIGH);
  pinMode(right_MS2, OUTPUT);
  digitalWrite(right_MS2, LOW);

  pinMode(left_enPin, OUTPUT);
  digitalWrite(left_enPin, LOW);
  pinMode(left_MS1, OUTPUT);
  digitalWrite(left_MS1, HIGH);
  pinMode(left_MS2, OUTPUT);
  digitalWrite(left_MS2, LOW);
  
  // Set up servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // Servo initialization
}

void loop() {
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
            //send_to_esp1("Hello from ESP2");
        }
    }

    // Small delay for better serial output handling
    //delay(100);
    command_handler(command);
    //send_to_esp1("Hello from ESP2");

}
