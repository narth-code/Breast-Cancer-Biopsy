
/* Texas A&M University 
** Created on: Feb 22, 2024   
** Author: Harry Tran
** File: main.cpp 
** -------- 
** Allows dynamic control of a stepper motor's speed, acceleration, and movement 
** through serial commands. Supports commands to set speed, acceleration, move to a specific angle,
** and continuously spin or stop the motor. This example is configured for a NEMA17 stepper motor
** with 1/16 microstepping.
**
**    loop() - Main loop handling serial commands to control the stepper motor.
*/

/* system includes */
#include <Arduino.h>

/* libraries files */
#include <AccelStepper.h>

/* local files */
#include <helperMotor.h>

// Initialize the stepper library on the pins
AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, stepPin, dirPin);

/**
 * @brief Setup routine to initialize serial communication and stepper motor.
 */
void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud.
  stepper.setCurrentPosition(0);
  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(6000); // steps per second
  stepper.setSpeed(100);
  stepper.setAcceleration(1000); // steps per second squared
}

/**
 * loop() - Listens for serial input and executes commands to control the motor's 
 * speed, acceleration, position, and to start or stop spinning.
 */
void loop() {
  bool spinning = false;
  float speed = 0;
  if (Serial.available()) // Check if data is available to read.
  { 
    String input = Serial.readStringUntil('\n'); // Read the input until a newline is received.
    if (input.startsWith("speed ")) 
    {
      float speed = input.substring(6).toFloat(); // Extract the speed value from the input.
      stepper.setSpeed(speed); // Set the stepper speed.
      Serial.print("--Speed set to ");
      Serial.println(speed);
    } 
    else  if (input.startsWith("set speed ")) 
    {
        float speed = input.substring(10).toFloat(); // Convert input to float.
        stepper.setMaxSpeed(speed);
        Serial.print("--New max speed set to ");
        Serial.println(speed);
    } 
    else  if (input.startsWith("set accel ")) 
    {
        float accel = input.substring(10).toFloat(); // Convert input to float.
        stepper.setAcceleration(accel);
        Serial.print("--New max acceleration set to ");
        Serial.println(accel);
    } 
    else if (input.startsWith("where")) 
    {
        Serial.print(">Current Position: ");
        Serial.println(stepper.currentPosition()); // Print the current position
    } 
    else if (input.startsWith("reset")) 
    {
        stepper.setCurrentPosition(0); // Reset the current position to 0
        Serial.println("--Position reset to 0.");
    } 
    else if (input.startsWith("spin")) 
    {
        // No target position, just spin continuously
        Serial.println("Spinning");
        stepper.move(speed);
        spinning = true;
    }
    else if (input.startsWith("stop")) 
    {
        // Stop spinning by setting speed to 0
        stepper.setSpeed(0);
        spinning = false;
    }
    else  if (input.startsWith("move ")) {
        float angle = input.substring(5).toFloat(); // Convert input to float.
        long targetSteps = (angle / 360.0) * NEMA17; // Convert angle to steps for 1/16 microstepping.
        
        Serial.print("Moving to ");
        Serial.print(angle);
        Serial.println(" degrees.");
        
        stepper.moveTo(targetSteps); // Set the target position.
    } 
    else{
        Serial.println("Command not found");
    }
  }
  stepper.run();
}
