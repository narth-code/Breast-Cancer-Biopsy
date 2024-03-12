
/* Texas A&M University 
** Created on: Mar 5, 2024   
** Author: Harry Tran
** File: main.cpp 
** -------- 

*/
#include <motorHelper.h>

#include <AccelStepper.h>



// Motor pin definitions
#define STEP_PIN_X 17
#define DIR_PIN_X 21
#define LIMIT_SWITCH_1 16 // Connect to the first limit switch for X
#define LIMIT_SWITCH_2 27 // Connect to the second limit switch for X

// Create the stepper motor object
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);

void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_SWITCH_1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);
  stepperX.setMaxSpeed(1000); // Set as needed
  stepperX.setAcceleration(500); // Set as needed

  calibrateAxis(stepperX, LIMIT_SWITCH_1, LIMIT_SWITCH_2);
}

void loop() {
  // Your loop code
}

void calibrateAxis(AccelStepper& stepper, int limitSwitch1, int limitSwitch2) {
  // Move towards the first limit switch
  while (digitalRead(limitSwitch1) == HIGH) {
    stepper.move(-1000);
    //stepper.moveTo(stepper.currentPosition() - 1); // Move stepper one step
    stepper.run();
  }
  Serial.println("First Limit Reached");
  stepper.stop(); // Stop the motor
  stepper.setCurrentPosition(0); // Reset the position to 0

  // Move away from the switch to ensure it's not triggered by mistake
  stepper.moveTo(100); // Small move away from the limit switch
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Move towards the second limit switch to find the maximum position
  while (digitalRead(limitSwitch2) == HIGH) {
    stepper.move(1000);
    //stepper.moveTo(stepper.currentPosition() + 1); // Move stepper one step
    stepper.run();
  }
  Serial.println("Second Limit Reached");
  stepper.stop(); // Stop the motor

  long maxPosition = stepper.currentPosition();
  Serial.print("Max position: ");
  Serial.println(maxPosition);

  // Optionally, you can set this as a limit or use it for calculations
  stepper.setCurrentPosition(0); // Reset position after calibration
}
