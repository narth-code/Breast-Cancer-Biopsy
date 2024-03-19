
/* Texas A&M University 
** Created on: Mar 5, 2024   
** Author: Harry Tran
** File: main.cpp 
** -------- 

*/
#include <Arduino.h>
#include <motorHelper.h>

#include <AccelStepper.h>



// Motor pin definitions
#define STEP_PIN_X 2
#define DIR_PIN_X 5
#define LIMIT_SWITCH_1 6 // Connect to the first limit switch for X
#define LIMIT_SWITCH_2 7 // Connect to the second limit switch for X


long maxPositionSteps = 44920; // This will store the calculated maximum steps after calibration
                       // Approximately 44920 steps


// Create the stepper motor object
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);

void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_SWITCH_1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);

  //Max Speed
  //Max Accel 
  stepperX.setMaxSpeed(2000); // Adjust based on your motor's characteristics
  stepperX.setAcceleration(600); // Start with a low value, then adjust

  //calibrateAxis(stepperX, LIMIT_SWITCH_1, LIMIT_SWITCH_2);
  //establishOrigin(stepperX, LIMIT_SWITCH_1);
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
    String input = Serial.readStringUntil('\r\n'); // Read the input until a newline is received.
    if (input.startsWith("speed ")) 
    {
      float speed = input.substring(6).toFloat(); // Extract the speed value from the input.
      stepperX.setSpeed(speed); // Set the stepper speed.
      Serial.print("--Speed set to ");
      Serial.println(speed);
    } 
    else  if (input.startsWith("set speed ")) 
    {
        float speed = input.substring(10).toFloat(); // Convert input to float.
        stepperX.setMaxSpeed(speed);
        Serial.print("--New max speed set to ");
        Serial.println(speed);
    } 
    else  if (input.startsWith("set accel ")) 
    {
        float accel = input.substring(10).toFloat(); // Convert input to float.
        stepperX.setAcceleration(accel);
        Serial.print("--New max acceleration set to ");
        Serial.println(accel);
    } 
    else if (input.startsWith("where")) 
    {
        Serial.print(">Current Position: ");
        Serial.println(stepperX.currentPosition()); // Print the current position
    } 
    else if (input.startsWith("reset")) 
    {
        stepperX.setCurrentPosition(0); // Reset the current position to 0
        Serial.println("--Position reset to 0.");
    } 
    else if (input.startsWith("spin")) 
    {
        // No target position, just spin continuously
        Serial.println("Spinning");
        stepperX.move(speed);
        spinning = true;
    }
    else if (input.startsWith("stop")) 
    {
        // Stop spinning by setting speed to 0
        stepperX.setSpeed(0);
        spinning = false;
    }
    else  if (input.startsWith("move ")) {
        float position = input.substring(5).toFloat(); // Convert input to float.
        long targetSteps = (position/100) * maxPositionSteps; // Convert angle to steps for 1/16 microstepping.
        
        Serial.print("Moving to ");
        Serial.print(position);
        Serial.println(" degrees.");
        
        stepperX.moveTo(-(targetSteps)); // Set the target position.
    } 
    else{
        Serial.println("Command not found");
    }
  }

  stepperX.run();
  if(stepperX.distanceToGo() == 0){
    Serial.println("Position Reached");
  }
}

void calibrateAxis(AccelStepper& stepper, int limitSwitch1, int limitSwitch2) {
  Serial.println(F("Calibration started."));

  // Approach the first limit switch
  // Move a large distance to ensure it hits the limit
  stepper.moveTo(100000); //Positive is moving towards the motor, rotates Clockwise
  while (digitalRead(limitSwitch1) == HIGH) {
    stepper.run();
  }
  Serial.println(F("First Limit Reached"));
  stepper.stop(); // Stop the motor
  stepper.setCurrentPosition(0); // Reset the position to 0
  stepper.move(-200); // Move away from the limit switch

  while (stepper.distanceToGo() != 0) {
    stepper.run(); // Clear the move
  }

  // Approach the second limit switch
  stepper.moveTo(-100000); // Move towards the other end
  while (digitalRead(limitSwitch2) == HIGH) {
    stepper.run();
  }
  Serial.println(F("Second Limit Reached"));
  stepper.stop(); // Stop the motor

  maxPositionSteps = -(stepper.currentPosition());
  Serial.print(F("Max position: "));
  Serial.println(maxPositionSteps);

  //stepper.setCurrentPosition(0); // Optionally reset position after calibration
  Serial.println(F("Calibration finished."));
}

void establishOrigin(AccelStepper& stepper, int limitSwitch1){
  Serial.println("Locating Origin.");
  stepper.moveTo(100000); //Positive is moving towards the motor, rotates CCW
  while (digitalRead(limitSwitch1) == HIGH) {
    stepper.run();
  }
  Serial.println("First Limit Reached");
  stepper.stop(); // Stop the motor
  stepper.setCurrentPosition(0); // Reset the position to 0

  maxPositionSteps = 44920;
  Serial.println("Origin set.");
}