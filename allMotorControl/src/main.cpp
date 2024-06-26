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

/* libraries files */
#include <AccelStepper.h>
#include <BluetoothSerial.h>

/* local files */
#include <testMotor.h>
#include <config.h>

BluetoothSerial SerialBT; //< Bluetooth Serial object for communication.

AccelStepper stepperX(AccelStepper::DRIVER, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);

Positions pos;  // Store the new positions of the motors
/**
 * @brief Setup function to initialize serial communication and motors.
 */
void setup()
{
  Serial.begin(BAUD);
  SerialBT.begin("ESP32_Control");
  Serial.println(F("Bluetooth Device is ready to pair"));
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
  bluetoothTask();

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
    stepperX.moveTo(pos.x);
    stepperY.moveTo(pos.y);
    stepperZ.moveTo(pos.z);
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
 * @brief Checks for available Bluetooth messages and parses them into motor commands.
 */
void bluetoothTask()
{
  String message;
  if (SerialBT.available())
  {
    message = SerialBT.readStringUntil('\n');

    Serial.print("Received: "); // Debugging: Print the received message
    Serial.println(message);

    // Parsing logic
    // Example parsing (very basic, for demonstration)
    float xPercent = strtol(strtok(&message[0], ","), NULL, 10);
    float yPercent = strtol(strtok(NULL, ","), NULL, 10);
    float zPercent = strtol(strtok(NULL, ","), NULL, 10);

    // Convert percentages to steps (0-100% -> 0-3200 steps)
    pos.x = (uint16_t)(xPercent / 100.0 * 3200);
    pos.y = (uint16_t)(yPercent / 100.0 * 3200);
    pos.z = (uint16_t)(zPercent / 100.0 * 3200);

    gotMessage = true;
  }
}


/**
 * @brief Initializes motor settings for maximum speed, acceleration, and initial position.
 */
void initializeMotors()
{

  stepperX.setCurrentPosition(0);
  stepperX.setMaxSpeed(6000);
  stepperX.setAcceleration(100);
  stepperX.setSpeed(200);

  stepperY.setCurrentPosition(0);
  stepperY.setMaxSpeed(6000);
  stepperY.setAcceleration(100);
  stepperY.setSpeed(200);

  stepperZ.setCurrentPosition(0);
  stepperZ.setMaxSpeed(6000);
  stepperZ.setAcceleration(100);
  stepperZ.setSpeed(200);
}

void calibrateAxis(AccelStepper& stepper, int limitSwitch1, int limitSwitch2) {
  uint16_t maxPositionSteps;
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