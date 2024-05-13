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
#include <EasyButton.h>

/* local files */
#include "testMotor.h"
#include "config.h"

BluetoothSerial SerialBT; //< Bluetooth Serial object for communication.

AccelStepper stepperX(AccelStepper::DRIVER, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);
AccelStepper *thisStepper = NULL;

EasyButton xLim(LIMIT_SWITCH_X, 2000, true, false);
EasyButton yLim(LIMIT_SWITCH_Y, 2000, true, false);
EasyButton zLim(LIMIT_SWITCH_Z, 50, true, true);

int i = 0;
float pos[3] = {0,0,0};  // Store the new positions of the motors
float maxSteps[3] = {0,0,0};
char mode = 'A';
/**
 * @brief Setup function to initialize serial communication and motors.
 */
void setup()
{
  Serial.begin(BAUD);
  #ifdef BLUETOOTH
    SerialBT.begin("ESP32_Motor_Control");
    Serial.println(F("Bluetooth Device is ready to pair"));
  #endif
  Serial.printf("ESP32 Motor Control, Version:%d \n", VERSION);
  initializeMotors();
  initializeButtons();

    thisStepper = &stepperY;
    calibrateAxis(thisStepper, &yLim);

    thisStepper = &stepperX;
    calibrateAxis(&stepperX, &xLim);
  #ifdef LIMIT_CALIBRATION
    thisStepper = &stepperX;
    calibrateAxis(thisStepper, LIMIT_SWITCH_X, LIMIT_SWITCH_X);
    thisStepper = &stepperY;
    calibrateAxis(thisStepper, LIMIT_SWITCH_Y, LIMIT_SWITCH_Y);
    thisStepper = &stepperZ;
    calibrateAxis(thisStepper, LIMIT_SWITCH_Z, LIMIT_SWITCH_Z);
  #endif
}

/**
 * @brief Main loop function, checks for Bluetooth messages and controls motors.
 */
void loop()
{
  serialTask();

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
    setSteppers();
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
  if(stepperX.distanceToGo() != 0){ stepperX.runSpeedToPosition();}
  if(stepperY.distanceToGo() != 0){ stepperY.runSpeedToPosition();}
  if(stepperZ.distanceToGo() != 0){ stepperZ.runSpeedToPosition();}

}


/** #MARK: serial
 * @brief Checks for available Bluetooth messages and parses them into motor commands.
 */
void serialTask()
{
  String message, data;
  
  #ifdef BLUETOOTH
    if (SerialBT.available()){
      message = SerialBT.readStringUntil('\n');

      Serial.print("Received: "); // Debugging: Print the received message
      Serial.println(message);
      gotMessage = true;
    }
  #else
    if (Serial.available() > 0) {
      String data = Serial.readStringUntil('\n'); // Read the incoming data until newline
      if ((data[0] == 'A' || data[0] == 'R')) {
        mode = data[0]; // Set mode to either Absolute or Relative
        Serial.println(mode == 'A' ? "Absolute Mode" : "Relative Mode");
      } 
      else {
        parseData(data);
        gotMessage = true;
      }
    }
  #endif
}


/**
 * @brief Initializes motor settings for maximum speed, acceleration, and initial position.
 */
void initializeMotors(){
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
//#MARK: BUTTONS
void buttonISR()
{
  xLim.read();
  yLim.read();
  zLim.read();

  // xLim.onPressed(limitXReached);
  // yLim.onPressed(limitYReached);
  // zLim.onPressed(limitZReached);
  
}
void initializeButtons(){
  //pinMode(LIMIT_SWITCH_X, INPUT_PULLUP);
  //pinMode(LIMIT_SWITCH_Y, INPUT_PULLUP);
  xLim.begin();
  yLim.begin();
  zLim.begin();
}

void setSteppers() {
  if (mode == 'A') { // Absolute positioning
    stepperX.moveTo(pos[X]*STEPS_PER_MM);
    stepperY.moveTo(pos[Y]*STEPS_PER_MM);
    stepperZ.moveTo(pos[Z]*STEPS_PER_MM);
  } else if (mode == 'R') { // Relative positioning
    stepperX.move(pos[X]*STEPS_PER_MM);
    stepperY.move(pos[Y]*STEPS_PER_MM);
    stepperZ.move(pos[Z]*STEPS_PER_MM);
  }
  stepperX.setSpeed(MOTOR_SPEED);
  stepperY.setSpeed(MOTOR_SPEED);
  stepperZ.setSpeed(MOTOR_SPEED);
}
//#MARK: PARSE
void parseData(String data) {
  int index1 = data.indexOf('X');
  int index2 = data.indexOf('Y');
  int index3 = data.indexOf('Z');

  if (index1 != -1) {
    int spaceIndex = data.indexOf(' ', index1);
    if (spaceIndex == -1) spaceIndex = data.length();
    pos[X] = data.substring(index1 + 1, spaceIndex).toFloat();
  }

  if (index2 != -1) {
    int spaceIndex = data.indexOf(' ', index2);
    if (spaceIndex == -1) spaceIndex = data.length();
    pos[Y] = data.substring(index2 + 1, spaceIndex).toFloat();
  }

  if (index3 != -1) {
    int spaceIndex = data.indexOf(' ', index3);
    if (spaceIndex == -1) spaceIndex = data.length();
    pos[Z] = data.substring(index3 + 1, spaceIndex).toFloat();
  }
}

//#MARK: calibrate
void calibrateAxis(AccelStepper* stepper, EasyButton* limitSwitch) {
Serial.println(F("Calibration started."));
  int speed = 4000;
  int delay = 0;
  // Approach the first limit switch
  // Move a large distance to ensure it hits the limit
  stepper->moveTo(-1000*STEPS_PER_MM); // Move towards the motor
  while (!(limitSwitch->isPressed())) {
    limitSwitch->read();
    stepper->setSpeed(speed);
    stepper->runSpeedToPosition();
    delayMicroseconds(delay);
  }
  Serial.println(F("First Limit Reached"));
  stepper->stop(); // Stop the motor
  stepper->setCurrentPosition(0); // Reset the position to 0
  stepper->moveTo(2*STEPS_PER_MM); // Move away from the limit switch  
  while (stepper->distanceToGo() != 0) {
    stepper->setSpeed(speed);
    stepper->runSpeedToPosition();
    delayMicroseconds(delay);
  }

  // Approach the second limit switch
  stepper->moveTo(1000*STEPS_PER_MM); //Positive is moving away the motor, rotates Clockwise
  while (!(limitSwitch->isPressed())) {
    limitSwitch->read();
    stepper->setSpeed(speed);
    stepper->runSpeedToPosition();
    delayMicroseconds(delay);
  }
  Serial.println(F("Second Limit Reached"));
  stepper->stop(); // Stop the motor
  Serial.printf("Axis steps, (mm): %d, (%2f)\n", abs(stepper->currentPosition()), 
                                        float(abs(stepper->currentPosition()) /STEPS_PER_MM));
  maxSteps[i++]= stepper->currentPosition();
//   Serial.print(F("Max steps: "));
//   Serial.println(abs(stepper->currentPosition()));

//   Serial.print(F("Axis Length (mm): "));
//   Serial.println((float)abs(stepper->currentPosition())/STEPS_PER_MM);
    
  
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