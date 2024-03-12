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

BluetoothSerial SerialBT; //< Bluetooth Serial object for communication.

AccelStepper stepperX(AccelStepper::DRIVER, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);

/**
 * @brief Setup function to initialize serial communication and motors.
 */
void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32_Control");
  Serial.println("Bluetooth Device is ready to pair");
  initializeMotors();
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
    stepperX.moveTo(xSteps);
    stepperY.moveTo(ySteps);
    stepperZ.moveTo(zSteps);
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
    xSteps = (long)(xPercent / 100.0 * 3200);
    ySteps = (long)(yPercent / 100.0 * 3200);
    zSteps = (long)(zPercent / 100.0 * 3200);

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