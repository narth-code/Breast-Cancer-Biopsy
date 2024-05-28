/* Texas A&M University 
** Created on: Feb 23, 2024   
** Author: Harry Tran
** File: main.cpp 
** -------- 
** This program sets up an ESP32 to control three stepper motors for x, y, and z axes,
** and communicates with a Bluetooth device to receive commands for motor control.
**
**    loop() - Main loop function, checks for Bluetooth messages and controls motors.
**    bluetoothTask() - changes the blink rate of LED1 from 1-10Hz based on user input.
*/

/* system includes */
#include <Arduino.h>

/* local helper files */
#define DEFINE_GLOBALS
#include "motorHelper.h"




/**
 * @brief Setup function to initialize serial communication and motors.
 */
void setup()
{
  Serial.begin(BAUD);
  initializeMotors();

  #ifdef LIMIT_CALIBRATION
    calibrateAxis(stepperX, LIMIT_SWITCH_X1, LIMIT_SWITCH_X2);
  #endif
}

/**
 * @brief Main loop function, checks for Bluetooth messages and controls motors.
 */
void loop()
{
  //checkSerial();

  if (gotMessage)
  {
    /* Set the target position. The run() function will try to move the motor 
     * (at most one step per call) from the current position to the target position 
     * set by the most recent call to this function. 
     * 
     * Caution: moveTo() also recalculates the speed for 
     * the next step. If you are trying to use constant 
     * speed movements, you should call setSpeed() after calling moveTo().
     */
  
    gotMessage = false;
  }
  /* Poll the motor and step it if a step is due, implementing 
   * accelerations and decelerations to achieve the target position. 
   * You must call this as frequently as possible, but at least once 
   * per minimum step time interval, preferably in your main loop. 
   * 
   * Note that each call to run() will make at most one step, 
   * and then only when a step is due,nbased on the current speed 
   * and the time since the last step.  
   */
  stepperX.run();
  stepperY.run();
  stepperZ.run();
}




/**
 * @brief Initializes motor settings for maximum speed, acceleration, and initial position.
 */
void initializeMotors()
{
  stepperX.setCurrentPosition(0);
  stepperX.setMaxSpeed(6000);
  stepperX.setSpeed(200);

  stepperY.setCurrentPosition(0);
  stepperY.setMaxSpeed(6000);
  stepperY.setSpeed(200);

  stepperZ.setCurrentPosition(0);
  stepperZ.setMaxSpeed(6000);
  stepperZ.setSpeed(200);
}
// MARK: CALIBRATE AXIS
void calibrateAxis(AccelStepper* stepper, int limitSwitch) {

  Serial.println(F("Calibration started."));
  int speed = 4000;
  short delay = 0;
  // Approach the first limit switch
  // Move a large distance to ensure it hits the limit
  stepper->moveTo(-1000*STEPS_PER_MM); // Move towards the motor
  while (digitalRead(limitSwitch) == LOW) {
    stepper->setSpeed(speed);
    stepper->runSpeedToPosition();
    delayMicroseconds(delay);
  }
  Serial.println(F("First Limit Reached"));
  stepper->stop(); // Stop the motor
  stepper->setCurrentPosition(0); // Reset the position to 0
  stepper->moveTo(4*STEPS_PER_MM); // Move away from the limit switch  
  while (stepper->distanceToGo() != 0) {
    stepper->setSpeed(speed);
    stepper->runSpeedToPosition();
    delayMicroseconds(delay);
  }

  // Approach the second limit switch
  stepper->moveTo(1000*STEPS_PER_MM); //Positive is moving away the motor, rotates Clockwise
  while (digitalRead(limitSwitch) == LOW) {
    stepper->setSpeed(speed);
    stepper->runSpeedToPosition();
    delayMicroseconds(delay);
  }
  Serial.println(F("Second Limit Reached"));
  stepper->stop(); // Stop the motor

  Serial.printf("Axis steps, (mm): %d, (%2f)\n", abs(stepper->currentPosition()), 
                                        float(abs(stepper->currentPosition()) /STEPS_PER_MM));


  //stepper->setCurrentPosition(0); // Optionally reset position after calibration
  Serial.println(F("Calibration finished."));

  stepper->moveTo(1*STEPS_PER_MM); // Move away from the limit switch  
  while (stepper->distanceToGo() != 0) {
    stepper->setSpeed(speed);
    stepper->runSpeedToPosition();
    delayMicroseconds(delay);
  }
  stepper->stop();
  stepper->disableOutputs();
}

